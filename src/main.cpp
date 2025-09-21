#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <ArduinoOTA.h>
#include <Adafruit_PWMServoDriver.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <cstring>

#include <U8g2lib.h>

#include "sensors.h"
#include "motion.h"
#include "line.h"
#include "main.h"
#include "comms.h"
#include "system/AudioFeedback.h"

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
#define Pump_Pin 12 //if I was 1% smarter I'd put this on 14 and the other on 12
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

//Timers
hw_timer_t *ShiftPWM_Handle = NULL;
hw_timer_t *SpeedRetrieval_Handle = NULL;


//Modes
int operationMode;
#define radioControlledMode 0
#define lineFollowMode 1
#define obstacleAvoidMode 2
#define compassOrientedObstacleAvoidMode 3
#define fireLookupMode 4
#define preProgrammedMode 5
#define calibrationMode 6
#define visionDrivenMode 7

ControlState controlState{};
StaticSemaphore_t controlStateMutexBuffer;
SemaphoreHandle_t controlStateMutex = nullptr;
TaskHandle_t sensorTaskHandle = nullptr;
TaskHandle_t controlTaskHandle = nullptr;
TaskHandle_t actuatorTaskHandle = nullptr;
TaskHandle_t uiTaskHandle = nullptr;

void resetControlState();
ControlState getControlStateSnapshot();
void updateBuzzerOutput(const ControlState &state);
void sensorTask(void *param);
void controlTask(void *param);
void actuatorTask(void *param);
void uiTask(void *param);

AudioFeedback audioFeedback([](uint16_t frequency) { ledcWriteTone(2, frequency); });
void drawStatusUi(uint32_t nowMs);

// ==================== Network configuration ====================
const char *WIFI_SSID = "Bulky Telemetry";
const char *WIFI_PASSWORD = "BulkyTelemetry";
const int TCP_PORT = 8000;

void resetControlState()
{
  if (controlStateMutex != nullptr) {
    xSemaphoreTake(controlStateMutex, portMAX_DELAY);
  }
  controlState.motion = STOP;
  controlState.speed = 0;
  controlState.pump = false;
  controlState.flash = false;
  controlState.buzzer = false;
  controlState.cameraMode = true;
  controlState.cameraYaw = 90;
  controlState.cameraPitch = 90;
  controlState.craneYaw = 90;
  controlState.cranePitch = 0;
  if (controlStateMutex != nullptr) {
    xSemaphoreGive(controlStateMutex);
  }
}

ControlState getControlStateSnapshot()
{
  ControlState snapshot{};
  if (controlStateMutex != nullptr) {
    if (xSemaphoreTake(controlStateMutex, portMAX_DELAY) == pdTRUE) {
      snapshot = controlState;
      xSemaphoreGive(controlStateMutex);
    }
  } else {
    snapshot = controlState;
  }
  return snapshot;
}

void updateBuzzerOutput(const ControlState &state)
{
  if (audioFeedback.isActive()) {
    return;
  }

  if (state.buzzer) {
    sound(1300);
  } else {
    sound(0);
  }
}

void drawStatusUi(uint32_t nowMs) {
  static uint32_t lastRenderMs = 0;
  if (nowMs - lastRenderMs < 250) {
    return;
  }
  lastRenderMs = nowMs;

  ControlState state = getControlStateSnapshot();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);

  u8g2.setCursor(0, 10);
  u8g2.print("Echo listener ready");

  u8g2.setCursor(0, 22);
  u8g2.print("SSID: ");
  u8g2.print(WIFI_SSID);

  u8g2.setCursor(0, 32);
  u8g2.print("Motion: 0x");
  u8g2.print(state.motion, HEX);

  u8g2.setCursor(0, 42);
  u8g2.print("Pump: ");
  u8g2.print(state.pump ? "ON" : "OFF");
  u8g2.print(" Flash: ");
  u8g2.print(state.flash ? "ON" : "OFF");

  u8g2.setCursor(0, 52);
  u8g2.print("Buzzer: ");
  u8g2.print(state.buzzer ? "ON" : "OFF");

  u8g2.setCursor(0, 62);
  if (state.cameraMode) {
    u8g2.print("Camera yaw/pitch: ");
    u8g2.print(state.cameraYaw);
    u8g2.print('/');
    u8g2.print(state.cameraPitch);
  } else {
    u8g2.print("Crane yaw/pitch: ");
    u8g2.print(state.craneYaw);
    u8g2.print('/');
    u8g2.print(state.cranePitch);
  }

  u8g2.sendBuffer();
}

void handleIncomingData()
{
  Comms::loop();
}

void action()
{
  if(operationMode==radioControlledMode)
  {

  }
}

void setup() {
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
  ledcSetup(2,20000,11); // 12-bit resolution can't achieve 20kHz with 80MHz clock


  ledcAttachPin(Pump_Pin,0);
  ledcAttachPin(LED_Pin,1);
  ledcAttachPin(Buzzer_Pin,2);

  //interrupt assignments
  // attachInterrupt(Encoder_Left_Top,LT_ISR,RISING);
  // attachInterrupt(Encoder_Left_Bot,LB_ISR,RISING);
  // attachInterrupt(Encoder_Right_Top,RT_ISR,RISING);
  // attachInterrupt(Encoder_Right_Bot,RB_ISR,RISING);


  Serial.begin(115200);
  setCpuFrequencyMhz(240);
  Serial.print("About To commence @ Frequency of : ");
  Serial.println(getCpuFrequencyMhz());
  delay(100);

  controlStateMutex = xSemaphoreCreateMutexStatic(&controlStateMutexBuffer);
  if (controlStateMutex == nullptr) {
    Serial.println("[SYS] Failed to allocate control state mutex");
  }
  resetControlState();

  ShiftPWM_Handle = timerBegin(0, 240, true);
  timerAttachInterrupt(ShiftPWM_Handle, &ShiftPWM, true);
  timerAlarmWrite(ShiftPWM_Handle, 3, true);
  timerAlarmEnable(ShiftPWM_Handle);

  // SpeedRetrieval_Handle = timerBegin(1, 240, true);
  // timerAttachInterrupt(SpeedRetrieval_Handle, &retreiveSpeeds, true);
  // timerAlarmWrite(SpeedRetrieval_Handle, 100000, true); //let's take a speed sample each 76ms maybe ? seems like a good compromise instinctively or something.
  // timerAlarmEnable(SpeedRetrieval_Handle);


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

  WiFi.disconnect(true);
  if (!Comms::init(WIFI_SSID, WIFI_PASSWORD, TCP_PORT)) {
    Serial.println("[COMMS] Failed to start echo listener");
  } else {
    Serial.println("[COMMS] Echo listener ready");
  }
  Serial.print("[COMMS] SoftAP SSID: ");
  Serial.println(WIFI_SSID);
  Serial.print("[COMMS] SoftAP IP: ");
  Serial.println(WiFi.softAPIP());

  ArduinoOTA.setHostname("Bulky");
  ArduinoOTA.onStart([]() {
    audioFeedback.playPattern(AudioFeedback::Pattern::TargetCleared);
  });
  ArduinoOTA.onEnd([]() {
    audioFeedback.playPattern(AudioFeedback::Pattern::TargetSelected);
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("[OTA] Error[%u]\n", error);
    audioFeedback.playPattern(AudioFeedback::Pattern::TelemetryTimeout);
  });
  ArduinoOTA.begin();

  if (xTaskCreatePinnedToCore(sensorTask, "sensor-task", 4096, nullptr, 3,
                              &sensorTaskHandle, 0) != pdPASS) {
    Serial.println("[SYS] Failed to start sensor task");
  }
  if (xTaskCreatePinnedToCore(controlTask, "control-task", 6144, nullptr, 4,
                              &controlTaskHandle, 1) != pdPASS) {
    Serial.println("[SYS] Failed to start control task");
  }
  if (xTaskCreatePinnedToCore(actuatorTask, "actuator-task", 4096, nullptr, 3,
                              &actuatorTaskHandle, 1) != pdPASS) {
    Serial.println("[SYS] Failed to start actuator task");
  }
  if (xTaskCreatePinnedToCore(uiTask, "ui-task", 4096, nullptr, 2,
                              &uiTaskHandle, 0) != pdPASS) {
    Serial.println("[SYS] Failed to start UI task");
  }
}


//experiment variables for before loop, optimize later.
unsigned long lastmillis;

unsigned long lastLineMillis;
unsigned long lastLineMillis2;
byte Oldline=0;
byte newLine;
double lastLineError;
double currentLineError;

void loop() {
  vTaskDelay(pdMS_TO_TICKS(100));
}

void sensorTask(void *param)
{
  (void)param;
  const TickType_t period = pdMS_TO_TICKS(5);
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    sense();
    getDistances();
    processBattery();
    processIRImissions();
    lineMode = 0;
    processLine();

    TickType_t now = xTaskGetTickCount();
    TickType_t elapsed = now - lastWake;
    if (elapsed < period) {
      vTaskDelayUntil(&lastWake, period);
    } else {
      lastWake = now;
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
}

void controlTask(void *param)
{
  (void)param;
  for (;;) {
    uint32_t now = millis();
    ArduinoOTA.handle();
    audioFeedback.loop(now);
    handleIncomingData();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void actuatorTask(void *param)
{
  (void)param;
  for (;;) {
    ControlState state = getControlStateSnapshot();
    projectMotion(state.motion, state.speed);
    pump(state.pump ? 4096 : 0);
    flash(state.flash ? 4096 : 0);
    updateBuzzerOutput(state);
    if (state.cameraMode) {
      camYaw(map(state.cameraYaw, 0, 180, 0, 90));
      camPitch(map(state.cameraPitch, 0, 180, 0, 90));
    } else {
      craneOffset(state.craneYaw);
      craneDeploy(state.cranePitch);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void uiTask(void *param)
{
  (void)param;
  for (;;) {
    drawStatusUi(millis());
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
