#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <SPI.h>
#include <Adafruit_PWMServoDriver.h>

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

<<<<<<< HEAD
//analog Data Retreival
static int analog_index;
static unsigned long analog_instance;
#define analog_interval 70 //in microseconds, maybe make it a variable if you want to be able to change this via the controller 
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
=======
>>>>>>> 327d605c805df3ffafb1cc00cf754745c26476eb




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

void loop() {
  // if(receive_Status){receive_Status=0;setMotion(reception.vector[0],reception.vector[1]);}
  sense(); //nothing shall be freezing, instead, work with instances and ticks and polling.
  getDistances();
  processBattery();
  processIRImissions();
  lineMode=0;
  processLine();
  projectMotion(reception.MotionState,reception.Speed);
  if(reception.bool1[0]){pump(4096);} else{pump(0);}
  if(reception.bool1[1]){flash(4096);} else{flash(0);}
  if(reception.bool1[2]){sound(1300);} else{sound(0);}
  if(reception.bool1[3])
  {
    camYaw(map(reception.yaw,0,180,0,90));
    camPitch(map(reception.pitch,0,180,0,90));
  }
  else
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
