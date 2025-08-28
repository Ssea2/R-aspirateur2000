#include <CytronMotorDriver.h>

#include "raspi_config.h"


static unsigned long lastMs = 0;
const unsigned long intervalMs = 20;


const int enca[] = {L_EncoderA, R_EncoderA};
const int encb[] = {L_EncoderB, R_EncoderB};
const int inA[] = {L_MotorA, R_MotorA};
const int inB[] = {L_MotorB, R_MotorB};

volatile int posi[] = {0,0};


float wheel_perimeter = PI*DIAM_WHEEL;
int speed_left;
int speed_right;
int speed = 60;
int dirL;
int dirR;
int target;
int dsec = 100; // mm
int Lldist;
int Rldist;

float move[] = {90,500}; // roatation en degrée et translation en mm


CytronMD motor_Left( PWM_PWM, inA[0], inB[0]);
CytronMD motor_Right(PWM_PWM, inA[1], inB[1]);

template <int j>
void readEncoder(){
  int b = digitalRead(encb[j]);
  if(b > 0){
    posi[j]--;
  }
  else{
    posi[j]++;
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
  return nbPulseAngle;
}

float getspeed(long pos, float dtime){
  float Covers = pos/dtime;
  float rpm = (Covers/nbPPR)*60.0;
  //float mps = (rpm*PI*(DIAM_WHEEL/1000))/60;
  return Covers;
}


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

bool iscollision(float dist_secu){
  int Lldist = getDistance(L_Distance);
  int Rldist = getDistance(R_Distance);
  if (Lldist < dist_secu || Rldist < dist_secu || getStop(L_Collision) || getStop(R_Collision)){
    return true;
  }
  else {
    return false;
  }
}


void setup() {
  Serial.begin(9600);

  pinMode(L_Collision , INPUT);
  pinMode(L_Distance  , INPUT);

  pinMode(R_Collision , INPUT);
  pinMode(R_Distance  , INPUT);


  for (int i=0; i<NBMOTOR; i++){
    pinMode(enca[i] , INPUT);
    pinMode(encb[i] , INPUT);
    pinMode(inA[i]  , OUTPUT);
    pinMode(inB[i]  , OUTPUT);
  /*pinMode(L_EncoderA  , INPUT);
  pinMode(L_EncoderB  , INPUT);
  pinMode(L_MotorB    , OUTPUT);
  pinMode(L_MotorA    , OUTPUT);

  pinMode(R_EncoderA  , INPUT);
  pinMode(R_EncoderB  , INPUT);
  pinMode(R_MotorB    , OUTPUT);
  pinMode(R_MotorA    , OUTPUT);*/
  }
    // Initialize Encoder
  attachInterrupt(digitalPinToInterrupt(L_EncoderA), readEncoder<0>, RISING);
  attachInterrupt(digitalPinToInterrupt(R_EncoderA), readEncoder<1>, RISING);

}

void loop(){  
  
  for (int i=0; i<2; i++){
    int action = move[i];
    if (action !=0){
      if (i==0){
        if (action > 0){
          dirL=-1;
          dirR=1;
        }
        else {
          dirL=1;
          dirR=-1;
        }
        target = angle2encodervalue(action);
      }
      else {
        dirL=1;
        dirR=1;
        target = dist2encodeurvalue(action);
      }

      noInterrupts();
      long cL = posi[0]; posi[0] = 0;
      long cR = posi[1]; posi[1] = 0;
      interrupts();
      speed_left = dirL*speed;
      speed_right = dirR*(COEFR2L*speed + BIASR2L);
      
      while (true){
        noInterrupts();
        cL = posi[0];
        cR = posi[1];
        interrupts();
        Serial.println(iscollision(dsec));
        if (fabs(cR) >= target) {
          break;  // on sort de la boucle
        }
        else if (iscollision(dsec)){
          motor_Left.setSpeed(0); motor_Right.setSpeed(0);
        }
        else {
          motor_Left.setSpeed(speed_left); motor_Right.setSpeed(speed_right);
        }
        
        
        
      }
      motor_Left.setSpeed(0); motor_Right.setSpeed(0);
      
    }
    else {
      Serial.println("STOP");
      motor_Left.setSpeed(0); motor_Right.setSpeed(0);
    }
  }

}