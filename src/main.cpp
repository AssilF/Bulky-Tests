#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Adafruit_PWMServoDriver.h>

#include <U8g2lib.h>

#include "sensors.h"
#include "motion.h"
#include "line.h"
#include "main.h"
#include "BulkyPackets.h"
#include "PeerRegistry.h"
#include "system/AudioFeedback.h"
#include "EspNowDiscovery.h"

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

ControlState controlState;
void resetControlState();
void updateControlFromComms();
void updateBuzzerOutput();

Comm::PeerRegistry peerRegistry;
EspNowDiscovery discovery(peerRegistry);
AudioFeedback audioFeedback([](uint16_t frequency) { ledcWriteTone(2, frequency); });
bool uiDirty = true;
uint32_t lastUiRenderMs = 0;

Comm::ControlPacket buildControlPacket(const ControlState &state);
void processPeerEvents();
void drawPeerUi(uint32_t nowMs);
String formatMac(const std::array<uint8_t, 6> &mac);
String linkStateToString(Comm::LinkState state);

void resetControlState()
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
  controlState.linkReady = false;
  controlState.targetMac.fill(0);
  controlState.lastTelemetryMs = 0;
}

void updateControlFromComms()
{
  controlState.linkReady = peerRegistry.hasTarget();
  if (controlState.linkReady) {
    auto mac = peerRegistry.getTarget();
    if (mac.has_value()) {
      controlState.targetMac = mac.value();
    }
  } else {
    controlState.targetMac.fill(0);
  }

  controlState.lastTelemetryMs = peerRegistry.lastTelemetryMs();

  if (controlState.linkReady) {
    Comm::ControlPacket packet = buildControlPacket(controlState);
    if (!discovery.sendControl(packet)) {
      Serial.println("[COMM] Failed to send control packet");
    }
  }
}

void updateBuzzerOutput()
{
  if (audioFeedback.isActive()) {
    return;
  }

  if (controlState.buzzer) {
    sound(1300);
  } else {
    sound(0);
  }
}

Comm::ControlPacket buildControlPacket(const ControlState &state) {
  Comm::ControlPacket packet;
  packet.header.magic = Comm::kPacketMagic;
  packet.header.version = Comm::kProtocolVersion;
  packet.header.type = Comm::MessageType::Control;
  packet.payload = Comm::encodeControlPayload(state.motion,
                                              state.speed,
                                              state.pump,
                                              state.flash,
                                              state.buzzer,
                                              state.cameraMode,
                                              state.cameraYaw,
                                              state.cameraPitch,
                                              state.craneYaw,
                                              state.cranePitch);
  return packet;
}

String formatMac(const std::array<uint8_t, 6> &mac) {
  char buffer[18];
  snprintf(buffer, sizeof(buffer), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buffer);
}

String linkStateToString(Comm::LinkState state) {
  switch (state) {
    case Comm::LinkState::Idle:
      return "Idle";
    case Comm::LinkState::Scanning:
      return "Scanning";
    case Comm::LinkState::Paired:
      return "Paired";
    case Comm::LinkState::Lost:
      return "Lost";
  }
  return "Unknown";
}

void processPeerEvents() {
  Comm::PeerEvent event;
  bool dirty = false;
  while (peerRegistry.popEvent(event)) {
    dirty = true;
    switch (event.type) {
      case Comm::PeerEvent::Type::ScanStarted:
        audioFeedback.playPattern(AudioFeedback::Pattern::ScanStart);
        Serial.println("[COMM] Scan started");
        break;
      case Comm::PeerEvent::Type::ScanStopped:
        audioFeedback.playPattern(AudioFeedback::Pattern::ScanStop);
        Serial.println("[COMM] Scan stopped");
        break;
      case Comm::PeerEvent::Type::PeerFound:
        audioFeedback.playPattern(AudioFeedback::Pattern::PeerFound);
        Serial.print("[COMM] Peer found: ");
        Serial.println(event.peer.name);
        break;
      case Comm::PeerEvent::Type::PeerUpdated:
        Serial.print("[COMM] Peer updated: ");
        Serial.println(event.peer.name);
        break;
      case Comm::PeerEvent::Type::PeerLost:
        audioFeedback.playPattern(AudioFeedback::Pattern::TargetCleared);
        Serial.print("[COMM] Peer lost: ");
        Serial.println(event.peer.name);
        break;
      case Comm::PeerEvent::Type::PeerAcked:
        audioFeedback.playPattern(AudioFeedback::Pattern::PeerAck);
        Serial.print("[COMM] Peer acknowledged: ");
        Serial.println(event.peer.name);
        break;
      case Comm::PeerEvent::Type::TargetSelected:
        audioFeedback.playPattern(AudioFeedback::Pattern::TargetSelected);
        Serial.print("[COMM] Target selected: ");
        Serial.println(event.peer.name);
        break;
      case Comm::PeerEvent::Type::TargetCleared:
        audioFeedback.playPattern(AudioFeedback::Pattern::TargetCleared);
        Serial.println("[COMM] Target cleared");
        break;
      case Comm::PeerEvent::Type::TelemetryReceived:
        controlState.lastTelemetryMs = peerRegistry.lastTelemetryMs();
        break;
      case Comm::PeerEvent::Type::TelemetryTimeout:
        audioFeedback.playPattern(AudioFeedback::Pattern::TelemetryTimeout);
        Serial.println("[COMM] Telemetry timeout");
        break;
    }
  }
  if (dirty) {
    uiDirty = true;
  }
}

void drawPeerUi(uint32_t nowMs) {
  if (!uiDirty && nowMs - lastUiRenderMs < 250) {
    return;
  }
  lastUiRenderMs = nowMs;
  uiDirty = false;

  auto peers = peerRegistry.peers();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setCursor(0, 10);
  u8g2.print("Peers:");

  for (size_t i = 0; i < peers.size() && i < 4; ++i) {
    const auto &peer = peers[i];
    bool isTarget = controlState.linkReady && controlState.targetMac == peer.mac;
    u8g2.setCursor(0, 22 + static_cast<uint8_t>(i) * 10);
    String label = peer.name.length() > 0 ? peer.name : formatMac(peer.mac);
    if (label.length() > 14) {
      label = label.substring(0, 14);
    }
    String prefix = isTarget ? "> " : "  ";
    if (peer.acknowledged) {
      prefix += "*";
    } else {
      prefix += " ";
    }
    u8g2.print(prefix + label);
  }

  u8g2.setCursor(0, 54);
  u8g2.print("Link: ");
  u8g2.print(linkStateToString(peerRegistry.getLinkState()));
  if (controlState.linkReady) {
    uint32_t ageMs = nowMs - controlState.lastTelemetryMs;
    u8g2.print(" ");
    u8g2.print(ageMs / 1000.0f, 1);
    u8g2.print("s");
  }

  if (controlState.linkReady) {
    u8g2.setCursor(0, 64);
    u8g2.print("Target: ");
    u8g2.print(formatMac(controlState.targetMac));
  }

  u8g2.sendBuffer();
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
  attachInterrupt(Encoder_Left_Top,LT_ISR,RISING);
  attachInterrupt(Encoder_Left_Bot,LB_ISR,RISING);
  attachInterrupt(Encoder_Right_Top,RT_ISR,RISING);
  attachInterrupt(Encoder_Right_Bot,RB_ISR,RISING);


  Serial.begin(115200);
  setCpuFrequencyMhz(240);
  Serial.print("About To commence @ Frequency of : ");
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
  Serial.println("Initializing wireless communications");
  delay(10);


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

  if (!discovery.begin()) {
    Serial.println("[COMM] Failed to initialize ESP-NOW discovery");
  }

  ArduinoOTA.setHostname("ilite-controller");
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

  uiDirty = true;
  processPeerEvents();
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
  uint32_t now = millis();
  ArduinoOTA.handle();
  processPeerEvents();
  audioFeedback.loop(now);

  sense(); //nothing shall be freezing, instead, work with instances and ticks and polling.
  getDistances();
  processBattery();
  processIRImissions();
  lineMode=0;
  processLine();
  updateControlFromComms();
  drawPeerUi(now);
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

}
