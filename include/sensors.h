#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

// Indexes for the multiplexed analog channels
#define battery_level 0
#define fire_detection_left 1
#define fire_detection_right 2
#define line_reading1 3
#define line_reading2 4
#define line_reading3 5
#define line_reading4 6
#define battery_status 7

// Array of analog sensor readings captured by sense().
extern int sensor_readings[];

// Latest measured distances in centimeters.
extern int front_distance;
extern int bot_distance;

// Battery level percentage calculated by processBattery().
extern int batteryLevel;

// Infrared bias used for fire detection.
extern int IRBias;

// Configurable infrared detection range.
extern int fireRange;

// Perform time-multiplexed analog sensing of all channels.
void sense();

// Map a floating point value from one range to another.
double mapdouble(double x, double in_min, double in_max, double out_min, double out_max);

// Retrieve ultrasonic distances for front and bottom sensors.
void getDistances();

// Compute battery level from raw ADC readings.
void processBattery();

// Determine fire direction using infrared sensors.
void processIRImissions();

#endif // SENSORS_H
