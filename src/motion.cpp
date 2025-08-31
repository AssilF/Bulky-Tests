#include "motion.h"
#include "sensors.h"

// Shift register control pins
#define SRdata 23
#define SRlatch 26
#define SRclock 27

// Motor bit masks
#define Motor_Left_Top B11000000
#define Motor_Left_Bot B00110000
#define Motor_Right_Top B00001100
#define Motor_Right_Bot B00000011

// Motion patterns
#define MOVE_FRONT B10101010
#define MOVE_BACK  B01010101
#define STOP       B00000000

#define LinearMotion 0
#define DynamicMotion 1
#define default_motion_state B00000000

#define Cm_per_Count_Conversion 1.0362
#define Cm_per_Second_Conversion 10.362
#define weight_iterations 5
#define EnableWeighting 0

static int PWMFreq_US=100;
static int MLT_PWM=0;
static int MLB_PWM=0;
static int MRT_PWM=0;
static int MRB_PWM=0;
static byte MotorPort=STOP;
byte MotionState=MOVE_FRONT;
static int ticks=0;
static bool MLTreset=1;
static bool MLBreset=1;
static bool MRTreset=1;
static bool MRBreset=1;
static int spinVar=0;
static bool MotionMode;

bool enablePIDReversal;
int balancedLeftSpeed;
int balancedRightSpeed;

// Motor weighting and encoder feedback -------------------------------------
static double MLT_Weight=1.000;
static double MLB_Weight=1.000;
static double MRT_Weight=1.000;
static double MRB_Weight=1.000;

static unsigned long LT_SPEED_COUNT;
static unsigned long LB_SPEED_COUNT;
static unsigned long RT_SPEED_COUNT;
static unsigned long RB_SPEED_COUNT;

static unsigned long LTlastMicros;
static unsigned long LBlastMicros;
static unsigned long RTlastMicros;
static unsigned long RBlastMicros;

static double MLT_WeightBuffer=0.000;
static double MLB_WeightBuffer=0.000;
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

short weightIteration=0;

void write_motor_register(byte movement)
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

void differentialDrive(int baseline_speed, int line_balance)
{
  MotionMode=DynamicMotion;
  if(enablePIDReversal)
  {
    balancedLeftSpeed=baseline_speed+(2*line_balance);
    balancedRightSpeed=baseline_speed-(2*line_balance);

    if(balancedLeftSpeed<0||balancedRightSpeed<0){
      MotionState= MOVE_BACK;}
    else if(balancedRightSpeed<0)
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
}

void IRAM_ATTR retreiveSpeeds()
{
  LT_SPEED_BUFFER = (LT_SPEED_COUNT-LT_SPEED_BUFFER);
  LB_SPEED_BUFFER = (LB_SPEED_COUNT-LB_SPEED_BUFFER)/2;
  RT_SPEED_BUFFER = (RT_SPEED_COUNT-RT_SPEED_BUFFER);
  RB_SPEED_BUFFER = (RB_SPEED_COUNT-RB_SPEED_BUFFER);

  LT_speed= (LT_SPEED_BUFFER)*Cm_per_Second_Conversion;
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
    MRB_WeightBuffer += (RB_SPEED_BUFFER/Raverage_count);
  }
  weightIteration++;
#endif

  LT_SPEED_BUFFER = LT_SPEED_COUNT;
  LB_SPEED_BUFFER = LB_SPEED_COUNT;
  RT_SPEED_BUFFER = RT_SPEED_COUNT;
  RB_SPEED_BUFFER = RB_SPEED_COUNT;
}

void projectMotion(byte Motion, byte speed)
{
  MotionState=Motion;
  MLT_PWM=speed;
  MLB_PWM=speed;
  MRT_PWM=speed;
  MRB_PWM=speed;
}

