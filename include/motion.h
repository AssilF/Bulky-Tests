#ifndef MOTION_H
#define MOTION_H

#include <Arduino.h>

// Current motion state written to the motor driver.
extern byte MotionState;

// Flag set by PID logic to indicate reverse motion.
extern bool enablePIDReversal;

// Drive the shift register with desired motor pattern.
void write_motor_register(byte movement);

// Timer interrupt for generating software PWM.
void IRAM_ATTR ShiftPWM();

// Set individual motor PWM values.
void setSpeeds(int speed, bool weighted);

// Compute PWM values for differential drive based on line balance.
void differentialDrive(int baseline_speed, int line_balance);

// Retrieve encoder counts and compute instantaneous speeds.
void IRAM_ATTR retreiveSpeeds();

// Encoder interrupt handlers.
void IRAM_ATTR LT_ISR();
void IRAM_ATTR LB_ISR();
void IRAM_ATTR RT_ISR();
void IRAM_ATTR RB_ISR();

// Apply a motion state with uniform speed to all motors.
void projectMotion(byte Motion, byte speed);

// Averaged encoder counts used for telemetry.
extern double average_count;

#endif // MOTION_H
