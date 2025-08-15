#include <CytronMotorDriver.h>

#include "raspi_config.h"
#include "raspi_motion.h"

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
  attachInterrupt(digitalPinToInterrupt(L_EncoderA), countL, RISING);
  attachInterrupt(digitalPinToInterrupt(R_EncoderA), countR, RISING);
}

void loop(){
  delay(10000);

  for (float i=25; i<128; i++){
    forward(i);
  }
  for (float i=128; i>25; i--){
    forward(i);
  }
  
}