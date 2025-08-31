#include "line.h"
#include "sensors.h"
#include "motion.h"

#define inverted_line 0
#define non_inverted_line 1
#define line_transducer_number 4

static int lineCalibrationArray[line_transducer_number];
int lineThresholdsUppers[line_transducer_number];
int lineThresholdsLowers[line_transducer_number];

bool lineMode;
bool fetchingLineFlag;
unsigned long lastLineFetchMillis;

void fetchLineThresholds(byte period)
{
  if(fetchingLineFlag){fetchingLineFlag=0; lastLineFetchMillis=millis();
  lineCalibrationArray[0]=sensor_readings[line_reading1];
  lineCalibrationArray[1]=sensor_readings[line_reading2];
  lineCalibrationArray[2]=sensor_readings[line_reading3];
  lineCalibrationArray[3]=sensor_readings[line_reading4];
  }
  else if(millis()-lastLineFetchMillis<=period*1000)
  {
  sensor_readings[line_reading1]>=lineCalibrationArray[0]? lineThresholdsUppers[0]=sensor_readings[line_reading1] : lineThresholdsUppers[0]=lineCalibrationArray[0];
  sensor_readings[line_reading2]>=lineCalibrationArray[1]? lineThresholdsUppers[1]=sensor_readings[line_reading2] : lineThresholdsUppers[1]=lineCalibrationArray[1];
  sensor_readings[line_reading3]>=lineCalibrationArray[2]? lineThresholdsUppers[2]=sensor_readings[line_reading3] : lineThresholdsUppers[2]=lineCalibrationArray[2];
  sensor_readings[line_reading4]>=lineCalibrationArray[3]? lineThresholdsUppers[3]=sensor_readings[line_reading4] : lineThresholdsUppers[3]=lineCalibrationArray[3];

  sensor_readings[line_reading1]<=lineCalibrationArray[0]? lineThresholdsLowers[0]=sensor_readings[line_reading1] : lineThresholdsLowers[0]=lineCalibrationArray[0];
  sensor_readings[line_reading2]<=lineCalibrationArray[1]? lineThresholdsLowers[1]=sensor_readings[line_reading2] : lineThresholdsLowers[1]=lineCalibrationArray[1];
  sensor_readings[line_reading3]<=lineCalibrationArray[2]? lineThresholdsLowers[2]=sensor_readings[line_reading3] : lineThresholdsLowers[2]=lineCalibrationArray[2];
  sensor_readings[line_reading4]<=lineCalibrationArray[3]? lineThresholdsLowers[3]=sensor_readings[line_reading4] : lineThresholdsLowers[3]=lineCalibrationArray[3];

  lineCalibrationArray[0]=sensor_readings[line_reading1];
  lineCalibrationArray[1]=sensor_readings[line_reading2];
  lineCalibrationArray[2]=sensor_readings[line_reading3];
  lineCalibrationArray[3]=sensor_readings[line_reading4];
  }
  else{fetchingLineFlag=1;}
}

#define line_right_most B00000001
#define line_mid_right B00000011
#define line_right B00000010
#define line_left_most B00001000
#define line_mid_left B00001100
#define line_left B00000100
#define line_target B00000110
#define line_blank B00000000

byte linePosition = B00000000;

byte processLine()
{
    linePosition=0;
    if(lineMode? sensor_readings[line_reading1]<=lineThresholdsLowers[0]:sensor_readings[line_reading1]>=lineThresholdsLowers[0])
    {
      linePosition|= B00000001;
    }
    if(lineMode? sensor_readings[line_reading2]<=lineThresholdsLowers[1]:sensor_readings[line_reading2]>=lineThresholdsLowers[1])
    {
      linePosition|= B00000010;
    }
    if(lineMode? sensor_readings[line_reading3]<=lineThresholdsLowers[2]:sensor_readings[line_reading3]>=lineThresholdsLowers[2])
    {
      linePosition|= B00000100;
    }
    if(lineMode? sensor_readings[line_reading4]<=lineThresholdsLowers[3]:sensor_readings[line_reading4]>=lineThresholdsLowers[3])
    {
      linePosition|= B00001000;
    }
    return linePosition;
}

int baseSpeed=20;
double kd=0.0005;
double kp=0.5;
static double lastDeviationErr;

double interpretPID(byte line, double lastErr)
{
  enablePIDReversal=0;
  double error;
  switch(line){
  case B00000110:
  error=0.00;
  break;
  case B00001111:
  error=0.00;
  break;
  case B00000000:
  error=lastErr/baseSpeed; enablePIDReversal=1;
  break;
  case B0000111:
  error=-2; lastDeviationErr=error;
  break;
  case B00000011:
  error=-1.5; lastDeviationErr=error;
  break;
  case B00000001:
  error=-3; lastDeviationErr=error;
  break;
  case B00001110:
  error=2; lastDeviationErr=error;

  break;
  case B00001100:
  error=1.5; lastDeviationErr=error;
  break;
  case B00001000:
    error=3; lastDeviationErr=error;
  break;
  default:
  error=lastDeviationErr;
  break;
  }
  error=((error-lastErr)*kd)+(error*kp);
  return  baseSpeed*error;
}

void interpretLine(byte Old_Line, byte Current_Line)
{
    if(Old_Line==B00000110 && Current_Line==B00000110 )
    {
      Serial.println("Straight");
      return;
    }
    if(Old_Line==B00000110 && Current_Line==B00001110 )
    {
      Serial.println("left");
      return;
    }
    if(Old_Line==B00000110 && Current_Line==B00000111 )
    {
      Serial.println("right");
      return;
    }
    if(Old_Line==B00000110 && Current_Line==B00000000 )
    {
      Serial.println("back");
      return;
    }
        if(Old_Line==B00001111 && Current_Line==B00000110 )
    {
      Serial.println("Cross");
      return;
    }
        if(Old_Line==B00001111 && Current_Line==B00000000 )
    {
      Serial.println("T");
      return;
    }
}

