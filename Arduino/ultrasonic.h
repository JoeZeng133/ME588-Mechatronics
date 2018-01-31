#include "utils.h"
#include "comm.h"

class ultrasonic
{
  static void isr0 ();
  static void isr1 ();
  
  int whichISR;
  static ultrasonic * instance0_;
  static ultrasonic * instance1_;
  
  void handleInterrupt ();
  
  volatile int counter_;

  int trg_pin, ech_pin;
  float a = 0.017; // speed of sound in cm/us (or whatever length units / us)
  unsigned long t0, t1;
  
  public:
    void begin_us (int whichISR_, int trg_pin_, int ech_pin_);
    void begin_us ();
    void ping ();

    bool complete;
    
};

void ultrasonic::begin_us (int whichISR_, int trg_pin_, int ech_pin_) {
  complete = 0;
  whichISR = whichISR_;
  trg_pin = trg_pin_;
  ech_pin = ech_pin_;
  pinMode(ech_pin, INPUT);
  pinMode(trg_pin, OUTPUT);
  digitalWrite(trg_pin, LOW);
  
  switch (whichISR) {
    case 0: 
      attachInterrupt (digitalPinToInterrupt(ech_pin), isr0, CHANGE); 
      instance0_ = this;
      break;
    case 1: 
      attachInterrupt (digitalPinToInterrupt(ech_pin), isr1, CHANGE); 
      instance1_ = this;
      break;
    }  
  }

void ultrasonic::isr0 () {
  instance0_->handleInterrupt ();  
}

void ultrasonic::isr1 () {
  instance1_->handleInterrupt ();  
}
  
ultrasonic * ultrasonic::instance0_;
ultrasonic * ultrasonic::instance1_;

void ultrasonic::handleInterrupt () {
  if(digitalRead(ech_pin)) {
    t0 = micros();
    complete = 0;
  }
  else {
    t1 = micros();
    state.US_dist[whichISR] = (t1-t0)*a;
    complete = 1;
  }
  }

void ultrasonic::ping() {
  //Serial.println(t0);
  digitalWrite(trg_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trg_pin, LOW);
}

