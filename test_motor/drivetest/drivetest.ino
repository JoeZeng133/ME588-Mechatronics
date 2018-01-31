#include "utils.h"

//int N;
unsigned long prevtime = 0, tot = 0;
int pinCh1 = 19, pinCh2 = 50;
int pinEna = 8;
int pinIn1 = 23, pinIn2 = 25;
int setpoint = 70;
int dead[2] = { -35, 35};

int pinPWM[2] = {5, 6};
int pinEnc[2] = {2, 18};
int pinDir[2] = {15, 16};

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
  // Serial.print(mot[0]->pid->u);
  //   Serial.print(" ");
  //   Serial.print(mot[1]->pid->u);
}

void loop() {

  if (millis() - prevtime > 10) {
    mot1->drive(setpoint);
    mot[0]->drive(setpoint);
    // Serial.print(mot[0]->pid->u);
    // Serial.print(" ");
    // Serial.print(mot[1]->pid->u);
    mot[1]->drive(setpoint);

//    Serial.print(mot[0]->pid->u);
//    Serial.print(" ");
//    Serial.print(mot[1]->pid->u);
     Serial.print(mot[0]->speed_motor);
     Serial.print(" ");
     Serial.print(mot[1]->speed_motor);
     Serial.print(" ");
    Serial.println(mot1->speed_motor);
    Serial.print("\n");
    prevtime = millis();
  }
}
