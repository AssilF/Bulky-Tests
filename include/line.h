#ifndef LINE_H
#define LINE_H

#include <Arduino.h>

// Detected line position encoded as bit mask.
extern byte linePosition;

// Selects inverted or non-inverted line mode.
extern bool lineMode;

// PID tuning parameters.
extern int baseSpeed;
extern double kd;
extern double kp;

// Calibration thresholds used for line detection.
extern int lineThresholdsLowers[4];

// Capture calibration samples for line sensors.
void fetchLineThresholds(byte period);

// Update linePosition based on current sensor readings.
byte processLine();

// Compute PID output given current line bit pattern.
double interpretPID(byte line, double lastErr);

// Provide human-readable interpretation of line transitions.
void interpretLine(byte Old_Line, byte Current_Line);

#endif // LINE_H
