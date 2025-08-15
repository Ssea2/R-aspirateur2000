#include <CytronMotorDriver.h>
#include <Arduino.h>

#include "raspi_config.h"
#include "raspi_motion.h"

int speed_left;
int speed_right;
long countL_nb;
long countR_nb;

float dist_between_wheel = 22.2;
float diameter_wheel = 70.0;

float wheel_perimeter = 2*PI*diameter_wheel; 


CytronMD motor_Left( PWM_PWM, L_MotorA, L_MotorB);
CytronMD motor_Right(PWM_PWM, R_MotorA, R_MotorB);

void countL() { countL_nb += 1; }
void countR() { countR_nb += 1; }


void forward(int speed){
  speed_left = speed;
  speed_right = (COEFR2L * speed) + BIASR2L;
  motor_Left.setSpeed(speed_left); motor_Right.setSpeed(speed_right);
  delay(100);
  Serial.print(speed);
  Serial.print(",");
  Serial.print(countL_nb);
  Serial.print(",");
  Serial.print(countR_nb);
  Serial.print("\n");
}

void backward(int speed){
  speed_left = speed;
  speed_right = (COEFR2L * speed) + BIASR2L;
  motor_Left.setSpeed(-speed_left); motor_Right.setSpeed(-speed_right);
  delay(100);
  Serial.print(speed);
  Serial.print(",");
  Serial.print(countL_nb);
  Serial.print(",");
  Serial.print(countR_nb);
  Serial.print("\n");
}

void turn_left(int speed){
  speed_left = speed;
  speed_right = (COEFR2L * speed) + BIASR2L;
  motor_Left.setSpeed(-speed_left); motor_Right.setSpeed(speed_right);
  delay(100);
  Serial.print(speed);
  Serial.print(",");
  Serial.print(countL_nb);
  Serial.print(",");
  Serial.print(countR_nb);
  Serial.print("\n");
}
void turn_right(int speed){
  speed_left = speed;
  speed_right = (COEFR2L * speed) + BIASR2L;
  motor_Left.setSpeed(speed_left); motor_Right.setSpeed(-speed_right);
  delay(100);
  Serial.print(speed);
  Serial.print(",");
  Serial.print(countL_nb);
  Serial.print(",");
  Serial.print(countR_nb);
  Serial.print("\n");
}

void stop(int speed){
  motor_Left.setSpeed(0); motor_Right.setSpeed(0);
  delay(100);
  Serial.print(speed);
  Serial.print(",");
  Serial.print(countL_nb);
  Serial.print(",");
  Serial.print(countR_nb);
  Serial.print("\n");
}