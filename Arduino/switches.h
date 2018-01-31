#include "utils.h"
#include "comm.h"

class LimitSwitch {
  public:
    int pin;
    int instance;
    bool currentState;

    void begin_sw (int PIN, int inst) {
      instance = inst;
      pin = PIN;
      pinMode(pin,INPUT);
      currentState = digitalRead(pin);
    }

    void update_sw() {
      bool a = digitalRead(pin);
      if (a && !currentState) {
        state.lim_stat[instance]++;
      }
      currentState = a;
    }

    void reset() {
      state.lim_stat[instance] = 0;
    }
};

class PhotoSwitch {
  public:
    int pin;
    int threshold;
    int instance;

    PhotoSwitch (int PIN, int inst) {
      instance = inst;
      pin = PIN;
      update_ps();
    }

    void update_ps() {
      state.phot_V[instance] = analogRead(pin);   
    }
    
};

