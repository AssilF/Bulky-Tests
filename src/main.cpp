#include <Arduino.h>
#include <WiFi.h>
#include <SPI.h>
#include <Adafruit_PWMServoDriver.h>
#include <ArduinoOTA.h>

#include <U8g2lib.h>

#include "sensors.h"
#include "motion.h"
#include "line.h"
#include "comms.h"

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

constexpr char WIFI_SSID[] = "Bulky Telemetry Port";
constexpr char WIFI_PASSWORD[] = "";
constexpr uint32_t COMMAND_TIMEOUT_MS = 500;
constexpr char OTA_HOSTNAME[] = "bulky-drone";
constexpr char OTA_PASSWORD[] = "";
constexpr uint16_t PAIRING_FEEDBACK_TONE_HZ = 1800;
constexpr uint32_t PAIRING_FEEDBACK_DURATION_MS = 200;
constexpr uint32_t PAIRING_FEEDBACK_PERIOD_MS = 1000;

struct ControlState {
  byte motion;
  byte speed;
  bool pump;
  bool flash;
  bool buzzer;
  bool cameraMode;
  uint8_t cameraYaw;
  uint8_t cameraPitch;
  uint8_t craneYaw;
  uint8_t cranePitch;
};

static ControlState controlState;
static uint32_t lastPairingAckHandled = 0;
static uint32_t pairingToneDeadline = 0;
static uint32_t nextPairingToneTime = 0;

static void resetControlState();
static void applyCommand(const Comms::ControlPacket &cmd);
static void updateControlFromComms();
static void initOTA();
static void updatePairingFeedback();
static void updateBuzzerOutput();

static void resetControlState()
{
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
}

static void applyCommand(const Comms::ControlPacket &cmd)
{
  controlState.motion = cmd.MotionState;
  controlState.speed = cmd.Speed;
  controlState.pump = cmd.bool1[0];
  controlState.flash = cmd.bool1[1];
  controlState.buzzer = cmd.bool1[2];
  controlState.cameraMode = cmd.bool1[3];
  controlState.cameraYaw = static_cast<uint8_t>(constrain(static_cast<int>(cmd.yaw), 0, 180));
  controlState.cameraPitch = static_cast<uint8_t>(constrain(static_cast<int>(cmd.pitch), 0, 180));
  controlState.craneYaw = controlState.cameraYaw;
  controlState.cranePitch = controlState.cameraPitch;
}

static void updateControlFromComms()
{
  Comms::ControlPacket command{};
  bool linked = Comms::receiveCommand(command);
  uint32_t lastCommand = Comms::lastCommandTimeMs();
  uint32_t age = lastCommand ? (millis() - lastCommand) : (COMMAND_TIMEOUT_MS + 1);

  if (linked && lastCommand != 0 && age <= COMMAND_TIMEOUT_MS) {
    applyCommand(command);
  } else {
    resetControlState();
  }
}

static void initOTA()
{
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  if (OTA_PASSWORD[0] != '\0') {
    ArduinoOTA.setPassword(OTA_PASSWORD);
  }

  ArduinoOTA.onStart([]() {
    Serial.println("OTA update starting");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("OTA update finished");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    if (total != 0) {
      uint32_t percent = (progress * 100U) / total;
      Serial.printf("OTA Progress: %u%%\r\n", percent);
    }
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]\n", static_cast<unsigned int>(error));
    if (error == OTA_AUTH_ERROR) {
      Serial.println("OTA Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("OTA Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("OTA Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("OTA Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("OTA End Failed");
    }
  });

  ArduinoOTA.begin();
  Serial.print("OTA Ready. IP address: ");
  Serial.println(WiFi.softAPIP());
}

static void updatePairingFeedback()
{
  uint32_t now = millis();
  uint32_t ackTime = Comms::lastPairingAckTimeMs();
  if (ackTime != 0 && ackTime != lastPairingAckHandled) {
    lastPairingAckHandled = ackTime;
    pairingToneDeadline = now + PAIRING_FEEDBACK_DURATION_MS;
    nextPairingToneTime = now + PAIRING_FEEDBACK_PERIOD_MS;
  }

  if (Comms::paired()) {
    nextPairingToneTime = 0;
    return;
  }

  if (pairingToneDeadline == 0) {
    if (nextPairingToneTime == 0 || static_cast<int32_t>(now - nextPairingToneTime) >= 0) {
      pairingToneDeadline = now + PAIRING_FEEDBACK_DURATION_MS;
      nextPairingToneTime = now + PAIRING_FEEDBACK_PERIOD_MS;
    }
  }
}

static void updateBuzzerOutput()
{
  if (controlState.buzzer) {
    sound(1300);
    pairingToneDeadline = 0;
    return;
  }

  if (pairingToneDeadline != 0) {
    uint32_t now = millis();
    if (static_cast<int32_t>(pairingToneDeadline - now) > 0) {
      sound(PAIRING_FEEDBACK_TONE_HZ);
      return;
    }
    pairingToneDeadline = 0;
  }

  sound(0);
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

  resetControlState();
  if (!Comms::init(WIFI_SSID, WIFI_PASSWORD, 0)) {
    Serial.println("Failed to initialise communications");
  } else {
    Serial.println("Communications initialised");
  }

  initOTA();



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

void loop() {
  sense(); //nothing shall be freezing, instead, work with instances and ticks and polling.
  getDistances();
  processBattery();
  processIRImissions();
  lineMode=0;
  processLine();
  updateControlFromComms();
  updatePairingFeedback();
  projectMotion(controlState.motion, controlState.speed);
  if(controlState.pump){pump(4096);} else{pump(0);} 
  if(controlState.flash){flash(4096);} else{flash(0);} 
  updateBuzzerOutput();
  if(controlState.cameraMode)
  {
    camYaw(map(controlState.cameraYaw,0,180,0,90));
    camPitch(map(controlState.cameraPitch,0,180,0,90));
  }
  else
  {
    craneOffset(controlState.craneYaw);
    craneDeploy(controlState.cranePitch);
  }

  Comms::sendTelemetry(Comms::PACK_TELEMETRY);
  ArduinoOTA.handle();
}
