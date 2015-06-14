/*
RC baot "brains"

some copy-paste from:
 - BaronPilot by Francesco Ferrara ferrara{a}libero.it
 
*/

static uint32_t rcTime;
static uint32_t currentTime;


#define DEADBAND 50
#define LEDPIN 13
#define SERIAL_COM_SPEED 115200
#define MAXCHANIN 4
#define CENTER 1500


int rawIn[MAXCHANIN];
float channelIn[MAXCHANIN];
float leftMotor=0;
float rightMotor=0;
float throttle=0;
float maxLimit=500;
float minLimit=-500;

// motor settings
int LM_1=12;
int LM_2=8;
int LM_pwm=10;

int RM_1=3;
int RM_2=9;
int RM_pwm=11;

void setup()
{ 
  Serial.begin(SERIAL_COM_SPEED);
  pinMode(LEDPIN,OUTPUT);
  setupMotors();
  setupRx();
}




void loop () {
  if (currentTime > (rcTime + 20000) ) { // 50Hz
	  updateRx();
    for(int i=0;i<MAXCHANIN;i++){
      Serial.print(channelIn[i]);
      Serial.print("\t");
  }
  throttle=(channelIn[2]/1000)*channelIn[1]; if (throttle!=0) { digitalWrite(LEDPIN,HIGH); } else { digitalWrite(LEDPIN,LOW); }
  leftMotor=throttle+channelIn[0]*(channelIn[2]/1000);
  rightMotor=throttle-channelIn[0]*(channelIn[2]/1000);
    maxLimit=max(leftMotor,rightMotor); if (maxLimit<500) maxLimit=500;
    minLimit=min(leftMotor,rightMotor); if (minLimit>-500) minLimit=-500;
    leftMotor=map(leftMotor,minLimit,maxLimit, -500,500);
    rightMotor=map(rightMotor,minLimit,maxLimit, -500,500);
          Serial.print(throttle);
        Serial.print("\t");
          Serial.print(leftMotor);
        Serial.print("\t");
              Serial.print(rightMotor);
  Serial.println("");
	  doMotors();
  rcTime = currentTime; 
  }
  currentTime = micros();
}






void setupMotors() {
  pinMode (LM_1, OUTPUT);
  pinMode (LM_2, OUTPUT);
  pinMode (LM_pwm, OUTPUT);
    digitalWrite (LM_1, LOW);
    digitalWrite (LM_2, LOW); 
    analogWrite(LM_pwm,0);
  pinMode (RM_1, OUTPUT);
  pinMode (RM_2, OUTPUT);
  pinMode (RM_pwm, OUTPUT);
    digitalWrite (RM_1, LOW);
    digitalWrite (RM_2, LOW); 
    analogWrite(RM_pwm,0);
}

void doMotors() {
  // use array!!!
  if (leftMotor>0) {
    digitalWrite (LM_1, HIGH);
    digitalWrite (LM_2, LOW); 
  } else if (leftMotor<0) {
    digitalWrite (LM_2, HIGH);
    digitalWrite (LM_1, LOW); 
  } else {// stop
    digitalWrite (LM_1, LOW);
    digitalWrite (LM_2, LOW); 
  }


  if (rightMotor>0) {
    digitalWrite (RM_1, HIGH);
    digitalWrite (RM_2, LOW); 
  } else if (rightMotor<0) {
    digitalWrite (RM_2, HIGH);
    digitalWrite (RM_1, LOW); 
  } else {// stop
    digitalWrite (RM_1, LOW);
    digitalWrite (RM_2, LOW); 
  }
  
  if (leftMotor!=0) {
    analogWrite(LM_pwm,map(abs(leftMotor),0,500,50,255));
  } else {// total stop
    analogWrite(LM_pwm,0);
  }

  if (rightMotor!=0) {
    analogWrite(RM_pwm,map(abs(rightMotor),0,500,50,255));
  } else {// total stop
    analogWrite(RM_pwm,0);
  }
  
}


void setupRx() {
  pinMode(2, INPUT); // 3 is used for esc
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  // interrupt on pin change PCINT
  PCICR |= (1 << PCIE2);

  PCMSK2 = (1 << PCINT18) | // pin2
  (1 << PCINT20) | // pin4
  (1 << PCINT21) | // pin5
  (1 << PCINT22) | // pin6
  (1 << PCINT23) ; // pin7 // not need if you don't use RXDIRECT
  for(int i=0;i<MAXCHANIN;i++)
    rawIn[i]=CENTER;
}

//***********************************************************************************************************************
byte newbit,oldbit,changed;
unsigned long startIn[MAXCHANIN];
unsigned long time;
int blockedRX=0;

#define MASKPCINT0 (1<<2)
#define MASKPCINT1 (1<<4)
#define MASKPCINT2 (1<<5)
#define MASKPCINT3 (1<<6)
#define MASKPCINT4 (1<<7)
ISR(PCINT2_vect)
{
  time=micros(); 

  newbit=PIND;

  // This is a new VERY VERY VERY fast method 
  // 1 xor operation 


  changed=newbit^oldbit;

  if (changed&MASKPCINT0)
    if (newbit&MASKPCINT0) startIn[0]=time;
    else rawIn[0]=time-startIn[0];

  if (changed&MASKPCINT1)
    if (newbit&MASKPCINT1) startIn[1]=time;
    else rawIn[1]=time-startIn[1];

  if (changed&MASKPCINT2)
    if (newbit&MASKPCINT2) startIn[2]=time;
    else rawIn[2]=time-startIn[2];

  if (changed&MASKPCINT3)
    if (newbit&MASKPCINT3) startIn[3]=time;
    else rawIn[3]=time-startIn[3];

  if (changed&MASKPCINT4)
    if (newbit&MASKPCINT4) startIn[4]=time;
    else rawIn[4]=time-startIn[4];

  oldbit=newbit;
  blockedRX=0;

}

void updateRx()
{
  if (blockedRX==0)
  {
    for(int i=0;i<MAXCHANIN;i++)
      if (rawIn[i]>800 && rawIn[i]<2200) {
        if (i<2) {
          channelIn[i]=(rawIn[i]-CENTER)*channelIn[3];
          // left/right and move
          if (channelIn[i]<DEADBAND && channelIn[i]>-DEADBAND) {
              channelIn[i]=0;
          }
        } else if (i==2) {
          // speed
          channelIn[i]=rawIn[i]-1000;
          if (channelIn[i]<DEADBAND) channelIn[i]=0;
          if (channelIn[i]>1000) channelIn[i]=1000;
          channelIn[i]=channelIn[i]*channelIn[3];  // speed=0 if disarmed
        } else if (i==3) {
          // disarm
          if (rawIn[i]>CENTER) {
            channelIn[i]=1;
          } else {
            channelIn[i]=0;
          }
        } else {
          channelIn[i]=0;
        }
      }
  }
  else if (blockedRX>20)
  {
    for(int i=0;i<MAXCHANIN;i++)
      channelIn[i]=0;
    blockedRX=0;
  }  

  blockedRX++;
}
























