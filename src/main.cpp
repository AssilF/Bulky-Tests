#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <SPI.h>
#include <Adafruit_PWMServoDriver.h>
#include <NewPing.h>

#include <U8g2lib.h>

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);
//this is the13th of october, my laptop apparently cannot even keep up with my rate of writing :D:D:D:
//notes:
// controlling motor speed and direction is performed by changing the MOTION STATE into one of the direction defs, Controlling each motor speed is performed by changing PWM vars, controlling overall bot speed is via a function.

//instances
Adafruit_PWMServoDriver servo_bus = Adafruit_PWMServoDriver(0x40);

//Servo Defs
#define ServoMinimalPW 125
#define ServoMaximalPW 575 //unsure of how this actually operate, but the pulse width is in terms of frequency it seems.

#define gripServo 0
#define hookServo 1
#define deployServo 2

#define sprayXServo 3
#define sprayYServo 4

//optinally implementable
#define chassisRaiseServo
#define lightDirServo
#define IrScanServo
#define camServo
#define SirenServo
#define lineIRDeployServo
#define US1Servo
#define US2Servo

//Verbose system
#define verboseEn 1
#if verboseEn == 1
#define debug(x) Serial.print(x);
#define verboseON Serial.begin(115200);
#else
#define debug(x) 
#define verboseON
#endif

//pin defs
#define LED_Pin 2
#define Gripper_Pin 36
#define Pump_Pin 12
#define Buzzer_Pin 14

#define SRdata 23 // MSB 00 LT, 00 LB, 00 RT, 00 RB
#define SRlatch 26
#define SRclock 27

#define Front_US_Echo 35
#define Front_US_Trig 16
#define Bot_US_Echo 34
#define Bot_US_Trig 17

#define Analog_Address_1 4
#define Analog_Address_2 33
#define Analog_Address_3 32
#define Analog_Reading_Pin 39

#define Encoder_Left_Top 25
#define Encoder_Left_Bot 13
#define Encoder_Right_Top 19
#define Encoder_Right_Bot 18



//motor defs
#define Motor_Left_Top B11000000 //Mask the target Motor only
#define Motor_Left_Bot B00110000
#define Motor_Right_Top B00001100
#define Motor_Right_Bot B00000011

#define MOVE_FRONT B10101010
#define MOVE_BACK B01010101
#define ROTATE_LEFT B10101111
#define ROTATE_RIGHT B11111010
#define TURN_LEFT B10100101
#define TURN_RIGHT B01011010
#define BREAK B11111111
#define STOP B00000000

//Servo Defs
#define craneDeployServo 0
#define craneShiftServo 1
#define camYawServo 2
#define camPitchServo 3

//function defs:
#define sound(hz) ledcWriteTone(2,hz)
#define note(note,octave) ledcWriteNote(2,note,octave)
#define pump(power) ledcWrite(0,power)
#define flash(power) ledcWrite(1,power)
#define camYaw(angle) servo_bus.setPWM(camYawServo,0,map(angle,0,180,ServoMinimalPW,ServoMaximalPW))
#define camPitch(angle) servo_bus.setPWM(camPitchServo,0,map(angle,0,180,ServoMinimalPW,ServoMaximalPW))
#define craneOffset(angle) servo_bus.setPWM(craneShiftServo,0,map(angle,0,180,ServoMinimalPW,ServoMaximalPW))
#define craneDeploy(angle) servo_bus.setPWM(craneDeployServo,0,map(angle,0,180,ServoMinimalPW,ServoMaximalPW))

//Global Vars
static int PWMFreq_US=100; //PWM frequency @ 100 microseconds yields 10KHz
static int MLT_PWM = 0; //PWM must be between 1 and 99
static int MLB_PWM = 0;
static int MRT_PWM = 0;
static int MRB_PWM = 0;
static byte MotorPort=STOP;
static byte MotionState=MOVE_FRONT;
static int ticks=0;
static bool MLTreset=1;
static bool MLBreset=1;
static bool MRTreset=1;
static bool MRBreset=1;
static int spinVar=0;
static bool MotionMode; 
#define LinearMotion 0
#define DynamicMotion 1
#define default_motion_state B00000000

//Timers
hw_timer_t *ShiftPWM_Handle = NULL;
hw_timer_t *SpeedRetrieval_Handle = NULL;

//analog Data Retreival
static int analog_index;
static unsigned long analog_instance;
#define analog_interval 7 //in microseconds, maybe make it a variable if you want to be able to change this via the controller 
#define battery_level 0
#define line_reading1 3 //right most
#define line_reading2 4
#define line_reading3 5
#define line_reading4 6 //left most
#define fire_detection_left 1
#define fire_detection_right 2
#define battery_status 7
static int sensor_readings[] = {0,0,0,0,0,0,0,0}; // 0 = battery, 1 = L1, 2 = L2, 3 = L3, 4 = L4, 5 = IR L, 6 = IR R, 7 = Batstat.

void sense()
{
  if(micros() - analog_instance >= analog_interval)
  {
    analog_instance=micros();
    sensor_readings[analog_index]= analogRead(Analog_Reading_Pin);
    analog_index++;if(analog_index>7){analog_index=0;}
    digitalWrite(Analog_Address_1, !(analog_index & 1));
    digitalWrite(Analog_Address_2, !((analog_index >> 1) & 1));
    digitalWrite(Analog_Address_3, !((analog_index >> 2) & 1)); //we're only interested in 8 bits of the 32 bits that make the esp32 word 
  }
}

double mapdouble(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//Functions
void write_motor_register(byte movement) //revise this code, maybe instead of the for loop, make it shift one bit at a time @ each timer interrupt cycle or something, I don't like spinning the CPU like that to satisfy the needs of a mere shift register. . .  
{
  for(byte i=0;i<8;i++)
  {
    digitalWrite(SRdata,movement & 1);
    digitalWrite(SRclock,1);
    while(spinVar<5){spinVar++;}spinVar=0;
    digitalWrite(SRclock,0);
    movement >>= 1;
  }
  digitalWrite(SRlatch,1);
  while(spinVar<5){spinVar++;}spinVar=0;
  digitalWrite(SRlatch,0);
}

//Motion ISRs
void IRAM_ATTR ShiftPWM()
{
  ticks++;
  if(ticks>=PWMFreq_US){write_motor_register(MotionState);ticks=0;MLTreset=1;MLBreset=1; MRTreset=1; MRBreset=1;MotorPort=MotionState;}
  else
  {
  if(ticks>=MLT_PWM&&MLTreset){MotorPort|= Motor_Left_Top;write_motor_register(MotorPort);MLTreset=0;}
  if(ticks>=MLB_PWM&&MLBreset){MotorPort|= Motor_Left_Bot;write_motor_register(MotorPort);MLBreset=0;}
  if(ticks>=MRT_PWM&&MRTreset){MotorPort|= Motor_Right_Top;write_motor_register(MotorPort);MRTreset=0;}
  if(ticks>=MRB_PWM&&MRBreset){MotorPort|= Motor_Right_Bot;write_motor_register(MotorPort);MRBreset=0;}
  }
}

//note: don't forget to make a function that checks if the motors are rotating at low PWM, if they ain't, spike the PWM to assist the motor to kickstart then lower PWM down to where it starts to move (for convenience)
static double MLT_Weight=1.000; //motors won't be rotating at the same speed right ?, we'll brute force them into rotating at the same speed by weighting each motor accordingly >:( , with data from the encoder.
static double MLB_Weight=1.000; //the weight is calculated via a transfer function or something between 0.1 and 1, I still haven't done the math yet
static double MRT_Weight=1.000;
static double MRB_Weight=1.000;

void setSpeeds(int speed, bool weighted)
{
  if(speed){
  speed = weighted ? constrain(speed,35,80):constrain(speed,20,100);
  MLT_PWM= weighted? speed+mapdouble(MLT_Weight,0.00,2.00,-20,20) : speed;
  MLB_PWM= weighted? speed-mapdouble(MLB_Weight,0.00,2.00,-20,20) : speed;

  MRT_PWM= weighted? speed+mapdouble(MRT_Weight,0.00,2.00,-20,20) : speed;
  MRB_PWM= weighted? speed-mapdouble(MRB_Weight,0.00,2.00,-20,20) : speed;
  }else
  {
    MotionState=default_motion_state;
  }
}

static bool enablePIDReversal;
int balancedLeftSpeed;
int balancedRightSpeed;

void differentialDrive(int baseline_speed, int line_balance)
{
  MotionMode=DynamicMotion;
  if(enablePIDReversal)
  {
  balancedLeftSpeed=baseline_speed+(2*line_balance);
  balancedRightSpeed=baseline_speed-(2*line_balance);

  if(balancedLeftSpeed<0||balancedRightSpeed<0){
  MotionState= MOVE_BACK;}else
  if(balancedRightSpeed<0)
  {
    MotionState=MOVE_FRONT;
  }
  }
  else{balancedLeftSpeed=baseline_speed+line_balance;
  balancedRightSpeed=baseline_speed-line_balance;
  MotionState= MOVE_FRONT;}

  MLT_PWM=constrain(balancedLeftSpeed,0,100);
  MLB_PWM=constrain(balancedLeftSpeed,0,100);

  MRT_PWM=constrain(balancedRightSpeed,0,100);
  MRB_PWM=constrain(balancedRightSpeed,0,100);
}

//Motion feedback
static unsigned long LT_SPEED_COUNT;
static unsigned long LB_SPEED_COUNT;//addition or substraction depends on the feedback from the accelerometer ? might introduce inacurracies since stuff goes through serial and this is an interrupt >:(
static unsigned long RT_SPEED_COUNT;//addition or substraction both depends on the accelerometer feedback + set direction :)
static unsigned long RB_SPEED_COUNT; //GOT IT ! implement drift detection via the accelerometer maybe ?, ikn the speed processing part, skip the drift parts for increased accuracy, maybe increase iteration density as well.

static unsigned long LTlastMicros;
static unsigned long LBlastMicros;
static unsigned long RTlastMicros;
static unsigned long RBlastMicros;

//Maybe Implement filters instead of deadtimes ?
void IRAM_ATTR LT_ISR(){ if(micros()-LTlastMicros>=400){
  LT_SPEED_COUNT++;
  LTlastMicros=micros();}
  }

void IRAM_ATTR LB_ISR(){
  if(micros()-LBlastMicros>=500){
  LB_SPEED_COUNT++;
  LBlastMicros=micros();}
  }

void IRAM_ATTR RT_ISR(){
  if(micros()-RTlastMicros>=500){
  RT_SPEED_COUNT++;
  RTlastMicros=micros();}
  }

void IRAM_ATTR RB_ISR(){
  if(micros()-RBlastMicros>=500){
  RB_SPEED_COUNT++;
  RBlastMicros=micros();}
  } //implement a timer to process the data or if else, implement a timing controlled polling system.



static double MLT_WeightBuffer=0.000; //motors won't be rotating at the same speed right ?, we'll brute force them into rotating at the same speed by weighting each motor accordingly >:( , with data from the encoder.
static double MLB_WeightBuffer=0.000; //the weight is calculated via a transfer function or something between 0.1 and 1, I still haven't done the math yet
static double MRT_WeightBuffer=0.000;
static double MRB_WeightBuffer=0.000;


static unsigned long LT_SPEED_BUFFER;
static unsigned long LB_SPEED_BUFFER;
static unsigned long RT_SPEED_BUFFER;
static unsigned long RB_SPEED_BUFFER;

double LT_speed;
double LB_speed;
double RT_speed;
double RB_speed;
double average_count = .000;
double Laverage_count = .000;
double Raverage_count = .000;
double totalTravel;
double totalLeftTravel;
double totalRightTravel;

#define weight_iterations 5
short weightIteration=0;

#define EnableWeighting 0


#define Cm_per_Count_Conversion 1.0362
#define Cm_per_Second_Conversion 10.362 //0.314*0.076*6.6*3.14 //18 degrees, corresponding to 0.314 pi 6.6 mm travelled, times 1/0.076 (76ms) (we get cm per second) //let's make it 50ms

void IRAM_ATTR retreiveSpeeds() //anyways, this step isn't really necessary but it's healthy for accuracy, so the bot doesn't count while doing calculations. . .
{ //LB is a different sensor from the rest.
  //we are taking speeds in reference to different positions @ 0.1s intervals

  LT_SPEED_BUFFER = (LT_SPEED_COUNT-LT_SPEED_BUFFER);
  LB_SPEED_BUFFER = (LB_SPEED_COUNT-LB_SPEED_BUFFER)/2;
  RT_SPEED_BUFFER = (RT_SPEED_COUNT-RT_SPEED_BUFFER);
  RB_SPEED_BUFFER = (RB_SPEED_COUNT-RB_SPEED_BUFFER);

  LT_speed= (LT_SPEED_BUFFER)*Cm_per_Second_Conversion;//taking the difference between travel @ last instance;
  LB_speed= (LB_SPEED_BUFFER)*Cm_per_Second_Conversion;
  RT_speed= (RT_SPEED_BUFFER)*Cm_per_Second_Conversion;
  RB_speed= (RB_SPEED_BUFFER)*Cm_per_Second_Conversion;

  Laverage_count= (LT_SPEED_BUFFER+LB_SPEED_BUFFER)/2;
  Raverage_count= (RT_SPEED_BUFFER+RB_SPEED_BUFFER)/2;
  average_count=(Laverage_count+Raverage_count)/2;


  #if EnableWeighting
    if(weightIteration >= weight_iterations)
  {
   MLT_Weight= MLT_WeightBuffer/weight_iterations;
   MLB_Weight= MLT_WeightBuffer/weight_iterations;
   MRT_Weight= MLT_WeightBuffer/weight_iterations;
   MRB_Weight= MLT_WeightBuffer/weight_iterations; 
   MLT_WeightBuffer = 0.000;
   MLB_WeightBuffer = 0.000;
   MRT_WeightBuffer = 0.000;
   MRB_WeightBuffer = 0.000;
   weightIteration=0;
  }

  if((MotionState==MOVE_FRONT||MotionState==MOVE_BACK)&&MotionMode==LinearMotion){
  MLT_WeightBuffer += (LT_SPEED_BUFFER/average_count);
  MLB_WeightBuffer += (LB_SPEED_BUFFER/average_count);
  MRT_WeightBuffer += (RT_SPEED_BUFFER/average_count);
  MRB_WeightBuffer += (RB_SPEED_BUFFER/average_count);
  }
  else{
  MLT_WeightBuffer += (LT_SPEED_BUFFER/Laverage_count);
  MLB_WeightBuffer += (LB_SPEED_BUFFER/Laverage_count);

  MRT_WeightBuffer += (RT_SPEED_BUFFER/Raverage_count);
  MRB_WeightBuffer += (RB_SPEED_BUFFER/Raverage_count);  //maybe implement a buffer if you wanna get the overall linear distance moved to compensate for error from these two only. 
  }
  weightIteration++;  
  }
  #endif

  LT_SPEED_BUFFER = LT_SPEED_COUNT;
  LB_SPEED_BUFFER = LB_SPEED_COUNT;
  RT_SPEED_BUFFER = RT_SPEED_COUNT;
  RB_SPEED_BUFFER = RB_SPEED_COUNT; //these buffers must 180 degrees out of phase with the speed counts @ the given time intervals
}


void projectMotion(byte Motion, byte speed) //currently, this function has no means of smoothly decelerating the bot before switching directions so the bot may get a G spike if u suddenly reverse direction, but we'll implement something in the extra time.
{
  MotionState=Motion;
   MLT_PWM=speed;
   MLB_PWM=speed;
   MRT_PWM=speed;
   MRB_PWM=speed;
}


//Distance Feedback
//library uses an independednt timer to drive the trigger pin so quickly the rail won't lose power.
#define frontSonar 0
#define botSonar 1

NewPing sonar[2] = { 
  NewPing(Front_US_Trig, Front_US_Echo), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(Bot_US_Trig, Bot_US_Echo)
};

int distanceIterations;
unsigned long pingInstance;
#define sonarPingInstance 2
int front_distance_buffer;
int bot_distance_buffer;
int front_distance;
int bot_distance;
bool is_front_distance_ready;
bool is_bot_distance_ready; //just having my freedom with ram out here :) 512KB is plenty for a reason.

void getDistances()
{
  if(millis()-pingInstance >= sonarPingInstance){
  if(distanceIterations<5){front_distance_buffer+=constrain(sonar[frontSonar].ping_cm(),0,45);}
  else if(distanceIterations==5){if(front_distance_buffer/5 != 0){front_distance=front_distance_buffer;} front_distance_buffer=0;}
  else if(distanceIterations<16){bot_distance_buffer+=constrain(sonar[botSonar].ping_cm(),0,30);}
  else if(distanceIterations==16){if(bot_distance_buffer/10 != 0){bot_distance=bot_distance_buffer/10;};bot_distance_buffer=0;}
  distanceIterations++;
  if(distanceIterations>16){distanceIterations=0;}
  pingInstance = millis();
  } //this is intensive on the cpu a bit, but at least we'd get less noise and false triggers polluting our ultrasonic flags.

} //I'll use this shit as is ou khlas.


#define inverted_line 0
#define non_inverted_line 1
bool lineMode;
#define line_transducer_number 4
#define calibrationSamples 7
#define calibrationTolerance //let's give it an arbitrary 400u of error at the time since the sensor has a steep curve
// static int lineCalibrationArray[line_transducer_number][calibrationSamples];
static int lineCalibrationArray[line_transducer_number];
static int lineThresholdsUppers[line_transducer_number];
static int lineThresholdsLowers[line_transducer_number];
// static int upper_calibration_buffer;
// static int lower_calibration_buffer;

// void fetchLineMode() //+ Calibration; 
// {//could be manual, could be automatic, I got no IDEA g ;)
//   for(int i=1;i<=line_transducer_number;i++) //this is like, what ? extra 4 clock cycles ? let it be, in case I ran out of flops, we remove this loop and turn it into flash torture 
//   {
//     for(int j=1;j<=calibrationSamples;j++)
//     {
//       lineThresholdsLowers[i]= lineCalibrationArray[i][j-1]>=lineCalibrationArray[i][j] ? lineCalibrationArray[i][j] : lineCalibrationArray[i][j-1]; 
//       lineThresholdsUppers[i]= lineCalibrationArray[i][j-1]<lineCalibrationArray[i][j] ? lineCalibrationArray[i][j] : lineCalibrationArray[i][j-1];
//     }}

//   //to figure out what line mode we have to add the linethresholds (lowers and upopers) for each transudcer and see which threshhold corresponds to an approximative gap.
//   //the approximative gap could (and I said "COULD") be calculated by: taking the average of the calibration buffer, (aka calibration buffer/transudcer_number) and subtracting the corresponding line threshhold +- some arbitrary tolerance.
//   //if one of the the calibration buffers' value isn't within the approximative gap and the other is within, you get your mode, upper = inverted, lower = non_inverted line mode.  
//   upper_calibration_buffer=lineThresholdsUppers[0];//sum everything up
//   lower_calibration_buffer=lineThresholdsLowers[0];
// } //this is a bulkier calibration array that'll be too difficult to implement since the competition is in 2 days

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
  sensor_readings[line_reading4]>=lineCalibrationArray[3]? lineThresholdsUppers[3]=sensor_readings[line_reading4] : lineThresholdsUppers[3]=lineCalibrationArray[3]; //this could be optimized further more but we only calibrate once so whatever.

  sensor_readings[line_reading1]<=lineCalibrationArray[0]? lineThresholdsLowers[0]=sensor_readings[line_reading1] : lineThresholdsLowers[0]=lineCalibrationArray[0];
  sensor_readings[line_reading2]<=lineCalibrationArray[1]? lineThresholdsLowers[1]=sensor_readings[line_reading2] : lineThresholdsLowers[1]=lineCalibrationArray[1];
  sensor_readings[line_reading3]<=lineCalibrationArray[2]? lineThresholdsLowers[2]=sensor_readings[line_reading3] : lineThresholdsLowers[2]=lineCalibrationArray[2];
  sensor_readings[line_reading4]<=lineCalibrationArray[3]? lineThresholdsLowers[3]=sensor_readings[line_reading4] : lineThresholdsLowers[3]=lineCalibrationArray[3];

  lineCalibrationArray[0]=sensor_readings[line_reading1];
  lineCalibrationArray[1]=sensor_readings[line_reading2];
  lineCalibrationArray[2]=sensor_readings[line_reading3];
  lineCalibrationArray[3]=sensor_readings[line_reading4];
  }
  else{fetchingLineFlag=1;} //make it motorized by rotating it left and right.
}

#define line_right_most B00000001
#define line_mid_right B00000011
#define line_right B00000010
#define line_left_most B00001000
#define line_mid_left B00001100
#define line_left B00000100
#define line_target B00000110
#define line_blank B00000000

#define line_differentiation_iterations 4
#define line_differentiation_duration 100

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

int baseSpeed=20; //
double kd=0.0005;
double kp=0.5;
static double lastDeviationErr;

double interpretPID(byte line, double lastErr) //recycled code from an old system.
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
  debug("\n error: ")
  debug(error)
  error=((error-lastErr)*kd)+(error*kp);
  return  baseSpeed*error;
}

void interpretLine(byte Old_Line, byte Current_Line) //you can control the bot behaviour depending on these interpretations, by enabling deviation or not.
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

#define full_battery_level 2491
 //R1 21KΩ, R2 4.9kΩ, //Full = 12.6v, depleted = 10.8v, with the voltage divider we implemented, we are well within range I guess
#define depleted_battery_level 2130 //Full charge = 2.384v ~ 3052, depleted = 2.04v ~ 2611, esp32 is sensitive up to 3.2v ~ 4096u, however, we need to compensate a drop of around 50mV due to the MUX action. around 640U
static int batteryLevel;

void processBattery() 
{
  batteryLevel=map(sensor_readings[battery_level],depleted_battery_level,full_battery_level,0,100); 
}

int IRBias;
int fireRange = 3000;//usually, at about 10 centimeters of fire the voltage drops considerable on the analogpin so.  .  .
#define FireTolerance //make these values modifiable by joystick

void processIRImissions()
{
  if(sensor_readings[fire_detection_left] <= fireRange || sensor_readings[fire_detection_right] <= fireRange)
  {
  IRBias = map(sensor_readings[fire_detection_left]-sensor_readings[fire_detection_right],-fireRange,fireRange,0,180); //let's give it angles of 180 for now.
  }
} //if this appears to be this simple we might as well combine all of this one one single data set ya za7. . .

//Modes
static int operationMode;
#define radioControlledMode 0
#define lineFollowMode 1
#define obstacleAvoidMode 2
#define compassOrientedObstacleAvoidMode 3
#define fireLookupMode 4
#define preProgrammedMode 5
#define calibrationMode 6
#define visionDrivenMode 7

void action()
{
  if(operationMode==radioControlledMode)
  {
    
  }
}


//COMS
uint8_t targetAddress[] = {0x78, 0x21, 0x84, 0x7E, 0x68, 0x1C}; //controller MAC 78:21:84:9A:58:28
static esp_now_peer_info bot;

static bool sent_Status;
static bool receive_Status;

//packet structs; these packets must NOT exceed 250bytes @ all costs!
//Sent Packets
struct receptionDataPacket
{  
  byte Speed;
  byte MotionState;
  byte pitch;
  byte yaw;
  bool bool1[4]; //corresponding to  the 3 buttons.
}reception; //dummy packet

struct emissionDataPacket
{
  byte INDEX;
  byte statusByte;
  int dataByte[8];
  byte okIndex;
}emission;

//Coms Fcns
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) 
{
  // Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&reception, incomingData, sizeof(reception));
  receive_Status=1;
}

void packData(byte index)
{
  switch(index)
  {
    case 0:
    emission.INDEX = index;
    emission.dataByte[0] = linePosition;
    emission.dataByte[1] = front_distance;
    emission.dataByte[2] = bot_distance;
    emission.dataByte[3] = IRBias;
    emission.dataByte[4] = average_count*Cm_per_Second_Conversion;
    emission.dataByte[5] = batteryLevel;
    emission.dataByte[6] = operationMode;
    break;

    case 1:
    emission.INDEX=index;
    emission.dataByte[0] = sensor_readings[line_reading1];
    emission.dataByte[1] = sensor_readings[line_reading2];
    emission.dataByte[2] = sensor_readings[line_reading3];
    emission.dataByte[3] = sensor_readings[line_reading4];
    emission.dataByte[4] = lineThresholdsLowers[0];
    emission.dataByte[5] = lineThresholdsLowers[1];
    emission.dataByte[6] = lineThresholdsLowers[2];
    emission.dataByte[7] = lineThresholdsLowers[3];
    break;

    case 2:
    emission.INDEX= index;
    emission.dataByte[0] = kp*100;
    emission.dataByte[1] = kd*100;
    emission.dataByte[3] = baseSpeed;
    break;

    case 3:
    emission.INDEX= index;
    emission.dataByte[0] = sensor_readings[fire_detection_left];
    emission.dataByte[1] = sensor_readings[fire_detection_right];
    emission.dataByte[3] = fireRange;
    break;

    default: 
    break;
  }
}

byte resendIndex;

void processData(byte index)
{
  
}

void setup() {
  verboseON
  u8g2.begin();
  //Pin Defs
  pinMode(SRdata,OUTPUT);
  pinMode(SRlatch,OUTPUT);
  pinMode(SRclock,OUTPUT);

  pinMode(Buzzer_Pin,OUTPUT);
  pinMode(Pump_Pin,OUTPUT); 
  pinMode(LED_Pin,OUTPUT);
  pinMode(Gripper_Pin,INPUT_PULLDOWN); //give it to address 1, use resistors.

  pinMode(Analog_Address_1,OUTPUT);  
  pinMode(Analog_Address_2,OUTPUT);
  pinMode(Analog_Address_3,OUTPUT);
  pinMode(Analog_Reading_Pin,INPUT);

  pinMode(Encoder_Left_Top,INPUT_PULLUP);
  pinMode(Encoder_Left_Bot,INPUT_PULLUP);
  pinMode(Encoder_Right_Top,INPUT_PULLUP);
  pinMode(Encoder_Right_Bot,INPUT_PULLUP);

  pinMode(Front_US_Trig,OUTPUT);
  pinMode(Bot_US_Trig,OUTPUT);
  pinMode(Front_US_Echo,INPUT); //give it to address 2, use resistors
  pinMode(Bot_US_Echo,INPUT_PULLDOWN); //give it to address 3, use resistors 

  //PWM drive defs
  ledcSetup(0,140,12); //pmp
  ledcSetup(1,10000,12); //LED, resulotion of 4096, might change if need to
  ledcSetup(2,20000,12);


  ledcAttachPin(Pump_Pin,0);
  ledcAttachPin(LED_Pin,1);
  ledcAttachPin(Buzzer_Pin,2);

  //interrupt assignments
  attachInterrupt(Encoder_Left_Top,LT_ISR,RISING);
  attachInterrupt(Encoder_Left_Bot,LB_ISR,RISING);
  attachInterrupt(Encoder_Right_Top,RT_ISR,RISING);
  attachInterrupt(Encoder_Right_Bot,RB_ISR,RISING);


  Serial.begin(115200);
  setCpuFrequencyMhz(240);
  Serial.print("Äbout To commence @ Frequency of : ");
  Serial.println(WiFi.macAddress());
  Serial.println(getCpuFrequencyMhz());
  delay(100);

  ShiftPWM_Handle = timerBegin(0, 240, true);
  timerAttachInterrupt(ShiftPWM_Handle, &ShiftPWM, true);
  timerAlarmWrite(ShiftPWM_Handle, 3, true);
  timerAlarmEnable(ShiftPWM_Handle);

  SpeedRetrieval_Handle = timerBegin(1, 240, true);
  timerAttachInterrupt(SpeedRetrieval_Handle, &retreiveSpeeds, true);
  timerAlarmWrite(SpeedRetrieval_Handle, 100000, true); //let's take a speed sample each 76ms maybe ? seems like a good compromise instinctively or something.
  timerAlarmEnable(SpeedRetrieval_Handle);



  //Init Wifi & ESPNOW ===============
  WiFi.mode(WIFI_STA); //just in case this is what helped with the uncought load prohibition exception.
  debug("\nwifi loaded\n");
  if (esp_now_init() != ESP_OK) {
  debug("ESPnow said no :(")
  }debug("\n\nespnow initialized\n")

  esp_now_register_send_cb(OnDataSent); debug("sending callback set \n \n")//Sent Callback Function associated
  memcpy(bot.peer_addr, targetAddress, 6); //putting the bot ID card into memory ;)
  bot.channel = 0;     
  bot.encrypt = false; //data won't be encrypted so we don't waste too much cpu juice

  if (esp_now_add_peer(&bot) != ESP_OK){
  debug("What the bot doin ?");
  }debug("peer added \n")

  esp_now_register_recv_cb(OnDataRecv); debug("reception callback set \n \n")//Recv Callback Function associated 
  //===================================



  //Servo/LED PWM BootUP  =============
  servo_bus.begin();
  servo_bus.setPWMFreq(60);
  //===================================

  //Default States  ===================
  MotionState = STOP;
  // craneDeploy(0);
  // craneOffset(0);
  camYaw(90);
  camPitch(90);
  //===================================

  //Bootup feedback ===================
  sound(300);
  flash(100);
  delay(100);
  sound(600);
  flash(800);
  delay(200);
  sound(720);
  flash(4096);
  delay(300);
  sound(0);
  flash(0);
  //==================================
  
  u8g2.setFont(u8g2_font_luBIS08_tr); // Set font

  u8g2.setCursor(45, 34);
  u8g2.print("VDrop");
  u8g2.sendBuffer();

}


//experiment variables for before loop, optimize later.
unsigned long lastmillis;

unsigned long lastLineMillis;
unsigned long lastLineMillis2;
byte Oldline=0;
byte newLine;
double lastLineError;
double currentLineError;
#define line_differentiation_sample_time 80 //80ms to differentiate the line
#define line_sample_time 1 //80ms to differentiate the line

void loop() {
  // if(receive_Status){receive_Status=0;setMotion(reception.vector[0],reception.vector[1]);}
  sense(); //nothing shall be freezing, instead, work with instances and ticks and polling.



  getDistances();
  processBattery();
  processIRImissions();
  lineMode=0;
  lineThresholdsUppers[0]=1000;
  lineThresholdsUppers[1]=1000;
  lineThresholdsUppers[2]=1000;
  lineThresholdsUppers[3]=1000;

  processLine();
   
  projectMotion(reception.MotionState,reception.Speed);
  if(reception.bool1[0]){pump(4096);}
  else{pump(0);}
  if(reception.bool1[1]){flash(4096);}
  else{flash(0);}
  if(reception.bool1[2]){sound(1300);}
  else{sound(0);}
  if(reception.bool1[3])
  {
    camYaw(map(reception.yaw,0,180,0,90));
    camPitch(map(reception.pitch,0,180,0,90));
  }else
  {
    craneOffset(reception.yaw);
    craneDeploy(reception.pitch);
  }

  // debug("\reception Speed: ")debug(reception.Speed);
  // debug("\reception MotionState: ")debug(String(reception.MotionState,BIN));
  packData(0);

  esp_err_t result = esp_now_send(targetAddress, (uint8_t *) &emission, sizeof(emission));
  if (result == ESP_OK) {
    
  }

}