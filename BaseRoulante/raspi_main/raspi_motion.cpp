/*#include <CytronMotorDriver.h>
#include <Arduino.h>

#include "raspi_config.h"
#include "raspi_motion.h"

int speed_left;
int speed_right;
float countL_nb;
float countR_nb;

float dist_between_wheel = 22.2;
float diameter_wheel = 70.0;

float wheel_perimeter = 2*PI*diameter_wheel; 


CytronMD motor_Left( PWM_PWM, L_MotorA, L_MotorB);
CytronMD motor_Right(PWM_PWM, R_MotorA, R_MotorB);

void countL() { countL_nb += 1; }
void countR() { countR_nb += 1; }


int forward(int speed){
  countL_nb=0;
  countR_nb=0;
  speed_left = speed;
  speed_right = (COEFR2L * speed) -2 ;
  motor_Left.setSpeed(speed_left); motor_Right.setSpeed(speed_right);
  for (int i; i< 1000; i++){
  //Serial.print(N);
  //Serial.print(",");
  //Serial.print(speed);
  //Serial.print(",");
  //Serial.print(speed_left);
  //Serial.print(",");
  //Serial.print(speed_right);
  //Serial.print(",");
    Serial.println(countL_nb);
    //Serial.print(",");
    //Serial.print(countR_nb);
  //Serial.print("\n");
    delay(1);
  }
  return 0;

  int target = 600;

  // PID constants
  float kp = 0.1;
  float kd = 0.025;
  float ki = 0.0;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  int pos = 0; 
  pos = posi;

  // error
  int e = pos - target;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 128 ){
    pwr = 128;
  }
  if( pwr < 25 ){
    pwr = 25;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  // signal the motor
  int speed = dir*pwr;
  motor_Left.setSpeed(speed);


  // store previous error
  eprev = e;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.print(" ");
  Serial.print(speed);
  Serial.println();
}*/
