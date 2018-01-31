#include "utils.h"


int N;
unsigned long prevtime = 0, tot = 0;
int pinCh1 = 2, pinCh2 = 3;
int pinEna = 6;
int pinIn1 = 8, pinIn2 = 9;
int setpoint = 70;
int dead[2] = {-35, 35};

int pinPWM[2] = {10, 11};
int pinEnc[2] = {20, 21};
int pinDir[2] = {52, 37};
// seqFilter<float> filt(2, 0.0);
motorPololu *mot1;
motorDigikey *mot[2];


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  motorGen::begin();
  mot1 = new motorPololu(pinCh1, pinCh2, pinEna, pinIn1, pinIn2);
  mot[0] = new motorDigikey(pinEnc[0], pinPWM[0], pinDir[0]);
  mot[1] = new motorDigikey(pinEnc[1], pinPWM[1], pinDir[1]);
  mot1->init();
  mot[0]->init();
  mot[1]->init();
  // analogWrite(pinEna, 0);
  // analogWrite(pinEna, 200);
  Serial.print("Current number of motors is");
  Serial.println(motorPololu::motorCount);
  // Serial.println(mot[1]->pinDir);

  while(Serial.available());
}

void loop() {
  // digitalWrite(37, LOW);
  // put your main code here, to run repeatedly
  if(Serial.available()){
	  float x, y, z;
	  // x = Serial.parseFloat();
	  // y = Serial.parseFloat();
	  // z = Serial.parseFloat();
	  setpoint = Serial.parseFloat();
	  // // mot1->pid->change(x, y, z);
	  // analogWrite(pinEna, 0);
	  // Serial.println("000000000000000000000");
	  // delay(1000);
	  mot1->refresh();
    mot[0]->refresh();
    mot[1]->refresh();
    prevtime = millis();

    // z = Serial.parseFloat();
    // analogWrite(pinEna, z);
  }

  if(millis() - prevtime > 10){
    mot1->drive(setpoint);
    mot[0]->drive(setpoint);
    mot[1]->drive(setpoint);
    // Serial.println(mot[1]->dir);
    // mot1->updateSpeed();
    // Serial.print(mot1->output);
    // Serial.print(" ");
    // Serial.print(mot1->speed);
    // Serial.print(",");
    // Serial.print(mot[0]->speed);
    // Serial.print(",");
    // Serial.println(mot[1]->speed);
    prevtime = millis();
  }
}
