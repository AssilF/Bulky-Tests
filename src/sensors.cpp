#include "sensors.h"
#include <NewPing.h>

// Hardware pin assignments
#define Analog_Address_1 4
#define Analog_Address_2 33
#define Analog_Address_3 32
#define Analog_Reading_Pin 39

#define Front_US_Echo 35
#define Front_US_Trig 16
#define Bot_US_Echo 34
#define Bot_US_Trig 17

// Analog sensor acquisition -------------------------------------------------
static int analog_index;
static unsigned long analog_instance;
#define analog_interval 7

int sensor_readings[] = {0,0,0,0,0,0,0,0};

void sense()
{
  if(micros() - analog_instance >= analog_interval)
  {
    analog_instance = micros();
    sensor_readings[analog_index] = analogRead(Analog_Reading_Pin);
    analog_index++; if(analog_index>7){analog_index=0;}
    digitalWrite(Analog_Address_1, !(analog_index & 1));
    digitalWrite(Analog_Address_2, !((analog_index >> 1) & 1));
    digitalWrite(Analog_Address_3, !((analog_index >> 2) & 1));
  }
}

double mapdouble(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Ultrasonic distance measurement ------------------------------------------
#define frontSonar 0
#define botSonar 1

NewPing sonar[2] = {
  NewPing(Front_US_Trig, Front_US_Echo),
  NewPing(Bot_US_Trig, Bot_US_Echo)
};

int distanceIterations;
unsigned long pingInstance;
#define sonarPingInstance 2
int front_distance_buffer;
int bot_distance_buffer;
int front_distance;
int bot_distance;

void getDistances()
{
  if(millis()-pingInstance >= sonarPingInstance){
    if(distanceIterations<5){front_distance_buffer+=constrain(sonar[frontSonar].ping_cm(),0,45);}
    else if(distanceIterations==5){if(front_distance_buffer/5 != 0){front_distance=front_distance_buffer;} front_distance_buffer=0;}
    else if(distanceIterations<16){bot_distance_buffer+=constrain(sonar[botSonar].ping_cm(),0,30);}
    else if(distanceIterations==16){if(bot_distance_buffer/10 != 0){bot_distance=bot_distance_buffer/10;} bot_distance_buffer=0;}
    distanceIterations++;
    if(distanceIterations>16){distanceIterations=0;}
    pingInstance = millis();
  }
}

// Battery and infrared processing -----------------------------------------
#define full_battery_level 2491
#define depleted_battery_level 2130

int batteryLevel;

void processBattery()
{
  batteryLevel = map(sensor_readings[battery_level], depleted_battery_level, full_battery_level, 0, 100);
}

int IRBias;
int fireRange = 3000;

void processIRImissions()
{
  if(sensor_readings[fire_detection_left] <= fireRange || sensor_readings[fire_detection_right] <= fireRange)
  {
    IRBias = map(sensor_readings[fire_detection_left]-sensor_readings[fire_detection_right], -fireRange, fireRange, 0, 180);
  }
}

