#include "ultrasonic.h"

const byte ech_pin = 2;
const byte trg_pin = 6;
unsigned long prevtime = 0;
ultrasonic US(0, trg_pin, ech_pin);

void setup() {
  Serial.begin(9600);
  US.begin();
}

void loop() {
  if(millis() - prevtime > 10){
    US.ping();
    prevtime = millis();
    if (US.complete) Serial.println(US.dist);
  }

}
