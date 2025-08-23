#include <CytronMotorDriver.h>

#include "raspi_config.h"


int speed_left;
int speed_right;
float countL_nb=0;
float countR_nb=0;
int pwr=0;

volatile int posi = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;

float wheel_perimeter = PI*DIAM_WHEEL; 


// Get distance from pololu sensor
int getDistance(int sensorPin) {
  int16_t t ;
  int flag = 0;
  while (flag == 0) {
    t = pulseIn(sensorPin, HIGH);
    if (t == 0) {
    } else if (t > 1850) {
      return 500;      
    } else {
      int16_t d = (t - 1000) * 3 / 4;
      if (d < 0) { d = 0; }
      return d;
    }
  }
}

// get Status from switch
int getStop(int sensorPin) {
  int val = analogRead(sensorPin);
  if (val <= 512) { return 0; } 
  else            { return 1; }
}



CytronMD motor_Left( PWM_PWM, L_MotorA, L_MotorB);
CytronMD motor_Right(PWM_PWM, R_MotorA, R_MotorB);

void countL(int dir) { 
  if (dir >=0) countL_nb += 1; }
void countR() { countR_nb += 1; }

void readEncoder(){
  int b = digitalRead(L_EncoderB);
  if(b > 0){
    posi--;
  }
  else{
    posi++;
  }
}

int dist2encodeurvalue(float distmm){
  float nbtour = 0;
  int nb_pulsation = 0;

  nbtour = distmm/wheel_perimeter;
  nb_pulsation = nbtour*nbPPR;
  return nb_pulsation;
}


float angle2encodervalue(float angle){
  float P_arc;
  int nbPulseAngle;
  // convertir un angle en distance
  P_arc= (PI*D_BETWEEN_WHEEL*angle)/360;
  nbPulseAngle = dist2encodeurvalue(P_arc);
  return P_arc;
}

void rotate(float angle, int speed=50){
  int dirL=1;
  int dirR=1;

  if (angle >0){
    dirL=-1;
    dirR=1;
  }
  else if (angle < 0){
    dirL=1;
    dirR=-1;
  }
  int target = dirL*angle2encodervalue(angle);
  speed_left = speed*dirL;
  speed_right = (COEFR2L * speed)*dirR;
  motor_Left.setSpeed(speed_left); motor_Right.setSpeed(speed_right);
  while (dirL*posi < target){
    int e = dirL*target - dirL*posi;
    Serial.print(e);
    Serial.print("||");
    Serial.print(posi);
    Serial.println();
  }
  delay(3000);
  motor_Left.setSpeed(0); motor_Right.setSpeed(0);
  }

void ForBackWard(float dist, int speed=50){
  int dir;
  if (dist > 0){
    dir = 1;
  }
  else{
    dir=-1;
  }
  int target = dir*dist2encodeurvalue(dist);
  Serial.println(target);
  delay(3000);
  speed_left = speed*dir;
  speed_right = (COEFR2L * speed)*dir;
  motor_Left.setSpeed(speed_left); motor_Right.setSpeed(speed_right);
  while (dir*posi < target){
    int e = dir*target - dir*posi;
    Serial.print(e);
    Serial.print("||");
    Serial.print(posi);
    Serial.println();
  }
  motor_Left.setSpeed(0); motor_Right.setSpeed(0);
  

}

void setup() {
  Serial.begin(9600);

  pinMode(L_EncoderA  , INPUT);
  pinMode(L_EncoderB  , INPUT);
  pinMode(L_MotorB    , OUTPUT);
  pinMode(L_MotorA    , OUTPUT);

  pinMode(R_EncoderA  , INPUT);
  pinMode(R_EncoderB  , INPUT);
  pinMode(R_MotorB    , OUTPUT);
  pinMode(R_MotorA    , OUTPUT);

    // Initialize Encoder
  attachInterrupt(digitalPinToInterrupt(L_EncoderA), readEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(R_EncoderA), countR, RISING);
}

void loop(){
  delay(3000);
  //rotate(90);//ForBackWard(100);
  //delay(1000);
  //rotate(-90);//ForBackWard(-100);
  Serial.println(dist2encodeurvalue(219.8));


}