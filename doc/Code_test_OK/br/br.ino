
///////////////////////////////////////////////////////////////
// Collision and Distance
// Left Sensor
#define L_Collision A0
#define L_Distance A1

// Right Sensor
#define R_Collision A2
#define R_Distance A3

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


///////////////////////////////////////////////////////////////
// Motor (lib from cytron) (PWM_PWM_DUAL)
#include "CytronMotorDriver.h"

// Pin motors Left
#define L_EncoderA 6
#define L_EncoderB 7
#define L_MotorB 8
#define L_MotorA 9

// Pin motors Right
#define R_EncoderA 2
#define R_EncoderB 3
#define R_MotorB 4
#define R_MotorA 5

CytronMD motor_Left( PWM_PWM, L_MotorA, L_MotorB);
CytronMD motor_Right(PWM_PWM, R_MotorA, R_MotorB);

/* // Use Motor
  motor_Left.setSpeed(128);   // Motor 1 runs forward at 50% speed.
  motor_Right.setSpeed(-128);  // Motor 2 runs backward at 50% speed.
  delay(1000);
  
  motor_Left.setSpeed(255);   // Motor 1 runs forward at full speed.
  motor_Right.setSpeed(-255);  // Motor 2 runs backward at full speed.
  delay(1000);

  motor_Left.setSpeed(0);     // Motor 1 stops.
  motor_Right.setSpeed(0);     // Motor 2 stops.
  */

long countL_nb = 0;
long countR_nb = 0;

void countL() { countL_nb += 1; }
void countR() { countR_nb += 1; }

///////////////////////////////////////////////////////////////
// Screen (lib Adafruit RECENTE)
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/* // Use Screen
  display.setCursor(0,0);
  display.println(F("Hello, world!"));
  display.display();
 */

///////////////////////////////////////////////////////////////
// Volt and Current reader (lib Rob Tillaart)
#include "INA226.h"
INA226 INA(0x40);

/* // Use Reader
    INA.getBusVoltage()
    INA.getShuntVoltage_mV()
    INA.getCurrent_mA()
    INA.getPower_mW()
 */

///////////////////////////////////////////////////////////////


void setup() {
  Serial.begin(9600);
  pinMode(L_Collision , INPUT);
  pinMode(L_Distance  , INPUT);
  pinMode(L_EncoderA  , INPUT);
  pinMode(L_EncoderB  , INPUT);
  pinMode(L_MotorB    , OUTPUT);
  pinMode(L_MotorA    , OUTPUT);

  pinMode(R_Collision , INPUT);
  pinMode(R_Distance  , INPUT);
  pinMode(R_EncoderA  , INPUT);
  pinMode(R_EncoderB  , INPUT);
  pinMode(R_MotorB    , OUTPUT);
  pinMode(R_MotorA    , OUTPUT);

  Serial.println("Initialisation");

  // Initialize Screen
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);

  // Initialize Monitor Current and Voltage
  Wire.begin();
  INA.begin();
  INA.setMaxCurrentShunt(1, 0.002);

  // Initialize Encoder
  attachInterrupt(digitalPinToInterrupt(L_EncoderA), countL, RISING);
  attachInterrupt(digitalPinToInterrupt(R_EncoderA), countR, RISING);

  // Print Everything is fine
  display.clearDisplay();
  display.setCursor(0,0);
  display.println(F("OK !"));
  display.display();
  delay(2000);
}







int tourneGauche(int degVoulu) {
  /*
   * Tourne le robot d'un certain degre à gauche
   * Retourne le nombre de degre fait :
   *  - identique à l'entrée si tout va bien
   *  - plus faible si collision
   */
   
  int degReel = 0;
  return degReel;
}

int tourneDroite(int degVoulu) {
  /*
   * Tourne le robot d'un certain degre à droite
   * Retourne le nombre de degre fait :
   *  - identique à l'entrée si tout va bien
   *  - plus faible si collision
   */
   
  int degReel = 0;
  return degReel;
}


int avance(int cmVoulu) {
  /*
   * Avance le robot d'un certain nombre de cm
   * Retourne le nombre de cm fait :
   *  - identique à l'entrée si tout va bien
   *  - plus faible si collision
   */
   
  int cmReel = 0;
  return cmReel;
}







void loop() {

  
  ////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////
  ////// Motor Test
  display.setTextSize(2);
  display.clearDisplay(); display.setCursor(0,0); display.println(F("Moteur !")); display.display();
  delay(1000);
  display.setTextSize(1);

  
  // Avance
  countL_nb = 0; countR_nb = 0;
  display.clearDisplay();
  display.setCursor(0,0); display.println(F("Avance :"));
  display.setCursor(0,10); display.println(F("Gauche :         "));
  display.setCursor(0,20); display.println(F("Droite :         "));
  display.display();
  motor_Left.setSpeed(128); motor_Right.setSpeed(128);
  for (int i=0;i<100;i++) {
    display.setCursor(50,10); display.println(countL_nb);
    display.setCursor(50,20); display.println(countR_nb);
    display.display();
    delay(1);
  }
  motor_Left.setSpeed(0); motor_Right.setSpeed(0);
  delay(100);

  // Recule
  countL_nb = 0; countR_nb = 0;
  display.clearDisplay();
  display.setCursor(0,0); display.println(F("Recule :"));
  display.setCursor(0,10); display.println(F("Gauche :         "));
  display.setCursor(0,20); display.println(F("Droite :         "));
  display.display();
  motor_Left.setSpeed(-128); motor_Right.setSpeed(-128);
  for (int i=0;i<100;i++) {
    display.setCursor(50,10); display.println(countL_nb);
    display.setCursor(50,20); display.println(countR_nb);
    display.display();
    delay(1);
  }
  motor_Left.setSpeed(0); motor_Right.setSpeed(0);
  delay(100);

  // Tourne à droite
  countL_nb = 0; countR_nb = 0;
  display.clearDisplay();
  display.setCursor(0,0); display.println(F("Droite :"));
  display.setCursor(0,10); display.println(F("Gauche :         "));
  display.setCursor(0,20); display.println(F("Droite :         "));
  display.display();
  motor_Left.setSpeed(128); motor_Right.setSpeed(-128);
  for (int i=0;i<100;i++) {
    display.setCursor(50,10); display.println(countL_nb);
    display.setCursor(50,20); display.println(countR_nb);
    display.display();
    delay(1);
  }
  motor_Left.setSpeed(0); motor_Right.setSpeed(0);
  delay(100);

  // Tourne à gauche
  countL_nb = 0; countR_nb = 0;
  display.clearDisplay();
  display.setCursor(0,0); display.println(F("Gauche :"));
  display.setCursor(0,10); display.println(F("Gauche :         "));
  display.setCursor(0,20); display.println(F("Droite :         "));
  display.display();
  motor_Left.setSpeed(-128); motor_Right.setSpeed(128);
  for (int i=0;i<100;i++) {
    display.setCursor(50,10); display.println(countL_nb);
    display.setCursor(50,20); display.println(countR_nb);
    display.display();
    delay(1);
  }
  motor_Left.setSpeed(0); motor_Right.setSpeed(0);
  delay(100);

  
  display.setTextSize(2);
  display.clearDisplay(); display.setCursor(0,0); display.println(F("Moteur Ok")); display.display();
  delay(1000);
  ////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////
  ////// Sensor Test
  display.setTextSize(2);
  display.clearDisplay(); display.setCursor(0,0); display.println(F("Switch !")); display.display();
  delay(1000);

  // Switch
  display.setTextSize(1);
  display.clearDisplay(); display.setCursor(0,0); display.println(F("Switch :")); display.display();

  display.setCursor(0,10); display.println(F("Gauche :")); display.display();
  display.setCursor(0,20); display.println(F("Droite :")); display.display();
  for (int i=0;i<200; i++) {
    if (getStop(L_Collision)) {
      display.setCursor(50,10); display.println(1);
    } else {
      display.setCursor(50,10); display.println(0);
    }
    if (getStop(R_Collision)) {
      display.setCursor(50,20); display.println(1);
    } else {
      display.setCursor(50,20); display.println(0);
    }
    display.display();
    delay(10);
  }

  // Lidar
  display.setTextSize(2);
  display.clearDisplay(); display.setCursor(0,0); display.println(F("Lidar !")); display.display();
  delay(1000);
  display.setTextSize(1);
  display.clearDisplay(); display.setCursor(0,0); display.println(F("Lidar :")); display.display();

  display.setCursor(0,10); display.println(F("Gauche :")); display.display();
  display.setCursor(0,20); display.println(F("Droite :")); display.display();
  for (int i=0;i<200; i++) {
    display.setCursor(50,10); display.println("     mm");
    display.setCursor(50,20); display.println("     mm");
    display.setCursor(50,10); display.println(getDistance(L_Distance));
    display.setCursor(50,20); display.println(getDistance(R_Distance));
    display.display();
    delay(10);
  }

  // INA226
  display.setTextSize(2);
  display.clearDisplay(); display.setCursor(0,0); display.println(F("INA 226 !")); display.display();
  delay(1000);
  display.setTextSize(1);
  display.clearDisplay(); display.setCursor(0,0); display.println(F("INA 226 :")); display.display();

  display.setCursor(0,10); display.println(F("Tension (V) :")); display.display();
  display.setCursor(0,20); display.println(F("Courant (A) :")); display.display();
  for (int i=0;i<200; i++) {
    display.setCursor(80,10); display.println("     ");
    display.setCursor(80,20); display.println("     ");
    display.setCursor(80,10); display.println(INA.getBusVoltage());
    display.setCursor(80,20); display.println(INA.getCurrent());
    display.display();
    delay(10);
  }

  display.setTextSize(2);
  display.clearDisplay(); display.setCursor(0,0); display.println(F("Sensors Ok")); display.display();
  delay(1000);
  ////////////////////////////////////////////////////////////////////////////////
  
}
