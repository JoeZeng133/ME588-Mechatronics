
class ultrasonic
{
  static void isr0 ();
  static void isr1 ();
  
  const byte whichISR;
  static ultrasonic * instance0_;
  static ultrasonic * instance1_;
  
  void handleInterrupt ();
  
  volatile int counter_;

  const byte trg_pin, ech_pin;
  const float a = 0.017; // speed of sound in cm/us (or whatever length units / us)
  unsigned long t0, t1;
  
  public:
    ultrasonic (const byte which_, const byte trg_pin_, const byte ech_pin_);
    void begin ();
    void ping ();

    float dist;
    bool complete;
    
};

void ultrasonic::begin () {
  complete = 0;
  
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
  
ultrasonic::ultrasonic (const byte whichISR_, const byte trg_pin_, const byte ech_pin_) : whichISR (whichISR_), trg_pin(trg_pin_), ech_pin(ech_pin_) {
    pinMode(ech_pin, INPUT);
    pinMode(trg_pin, OUTPUT);
    digitalWrite(trg_pin, LOW);
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
    dist = (t1-t0)*a;
    complete = 1;
  }
  }

void ultrasonic::ping() {
  //Serial.println(t0);
  digitalWrite(trg_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trg_pin, LOW);
}

