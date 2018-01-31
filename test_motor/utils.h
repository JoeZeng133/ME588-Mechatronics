// average filter >>>>>>>>>>>>>>>>>>>>>>>>>>>>

template <typename dataStream>
class seqFilter{

  private:
	dataStream *data, tot;
	int idx, N, Nbit;

 public:
	seqFilter(int N, dataStream const &init);
	void refresh(dataStream const &init);
  dataStream getVal();
};

template<typename dataStream>
seqFilter<dataStream>::seqFilter(int _N, dataStream const &init):Nbit(_N){
  N = 1<<_N;
  data = new dataStream[N];
  memset(data, 0, sizeof(dataStream) * N);
  idx = tot = data[0] = 0;
}

template<typename dataStream>
void seqFilter<dataStream>::refresh(dataStream const &init){
  tot += init;
  tot -= data[idx];
  data[idx ++] = init;
  if(idx == N) idx = 0;
}

template<typename dataStream>
dataStream seqFilter<dataStream>::getVal(){
  return tot >> Nbit;
}


// PID control >>>>>>>>>>>>>>>>>>>>>>>>>>>>
class PID{
  private:
	//they are floats
	float Kp, Ki, Kd, tmp;
	float ek, ek1, ek2, u, dt;
	//they are integers
	int deadBand[2], range[2], flags;

 public:
	PID(float const &Kp, float const &Ki, float const &Kd, int const &flags = 1, int *deadBand = nullptr, int *range = nullptr){

    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    // this->dt = dt;
    this->tmp = this->Ki;
    // Serial.print("the range is ");
    // Serial.print(range[0]);
    // Serial.print(" ");
    Serial.println(range[1]);
    if(deadBand != nullptr){

      this->deadBand[0] = deadBand[0];
      this->deadBand[1] = deadBand[1];
    }
    if(range != nullptr){

      this->range[0] = range[0];
      this->range[1] = range[1];
    }
    
    this->flags = flags;
    ek1 = ek2 = u = ek = 0;
  }

	void update(float const &setpoint, float const &input, float const &dt, int &res){
    ek2 = ek1;
    ek1 = ek;
    ek = setpoint - input;
    
    // Serial.print("the error");
    // Serial.print(ek);
    // Serial.print(", ");

    u += Kp * (ek - ek1) + Ki * ek * dt + Kd / dt * (ek - 2 * ek1 + ek2);
    
    //dead band zone
    res = u;
    // Serial.print("original is ");
    // Serial.print(ek);
    // Serial.print(",  ");
    // Serial.println(res);
    // Serial.print(", ");
    
    if(flags & 4){
      if(res > 0) res += deadBand[1];
      else res -= deadBand[0];
    }
    // saturation
    if(flags & 1){
      if(res > range[1]) res = range[1];
      if(res < range[0]) res = range[0];
      //anti-windup
      if(flags & 2){
        if(res == range[1] || res == range[0]){
          this->Ki = 0;
        }else{
          this->Ki = this->tmp;
        }
      }
    }
  }

	void change(float const &_Kp, float const &_Ki, float const &_Kd){
		Kp = _Kp;
		Ki = _Ki;
		Kd = _Kd;
	}

  void refresh(){
    u = ek = ek1 = ek2 = 0;
  }
};

// define general motors

class motorGen{
public:
  //static members for glue interrupt service routines, at most three instances for now
  static int motorCount;
  static void isr0();
  static void isr1();
  static void isr2();
  static motorGen * instance[3];
  static void (*isr[3])();
  static void begin();
  virtual void increment(); //called using father class, needs to be virtual
  motorGen(){};
};

motorGen (* motorGen::instance[3]);
void motorGen::isr0(){instance[0]->increment();}
void motorGen::isr1(){instance[1]->increment();}
void motorGen::isr2(){instance[2]->increment();}
int motorGen::motorCount = 0;
void (* motorGen::isr[3])();
void motorGen::begin(){
  isr[0] = isr0;
  isr[1] = isr1;
  isr[2] = isr2;
}

// define motor from digikey >>>>>>>>>>>>>>>>>>>>>>>>>>>>
#define speedRatio 111111.1111 * 0.75
#define filtBitNum 2
#define filtNum 3

class motorDigikey : public motorGen{
public:
  int range[2] = {0, 255};
  int pinEnc, pinPWM, pinDir, whichMotor, output;
  float speed;
  unsigned long prevMillis, nowMillis;
  bool flag = 0, dir = 1, setdir, stop;
  PID *pid;
  //Kp = 0.6, Ki = 12, Kd = 0.0002 is found to be perfect for this motor
  
  //unsigned long doesn't work, don't know why
  long tt, tmp, timeStamp[1 << filtBitNum];
  int idx;
  bool lockinter;
  // seqFilter<long> *filt;

  motorDigikey(int const &_pinEnc, int const &_pinPWM, int const &_pinDir) : pinEnc(_pinEnc), pinPWM(_pinPWM), pinDir(_pinDir){
    pinMode(pinEnc, INPUT);
    pinMode(pinPWM, OUTPUT);
    pinMode(pinDir, OUTPUT);
    digitalWrite(pinPWM, 255);
    digitalWrite(pinDir,1);
    //add to the stack of motor pointers for use of glue isr;
    whichMotor = motorCount;
    instance[motorCount++] = this;
    // Serial.print("The number of motor is ");
    // Serial.println(whichMotor);
    pid = nullptr;
    speed = prevMillis = nowMillis = 0;
  }

  void init(float Kp = 0.6, float Ki = 12, float Kd = 0.002, int flags = 3, int *deadBand = nullptr){
    // filt = new seqFilter<long>(2, 0);
    idx = 0;
    lockinter = 0;
    memset(timeStamp, 0, 4 * sizeof(long));
    Serial.println("Initialize motor PID controll");
    pid = new PID(Kp, Ki, Kd, flags, deadBand, range);
    attachInterrupt(digitalPinToInterrupt(pinEnc), isr[whichMotor], CHANGE);
  }

  void drive(float const &setpoint){
    //flags
    // static bool flag = 0, dir = 1, setdir, stop;
    // Serial.println(dir);
    updateSpeed();
    setdir = (setpoint > 0);
    stop = (speed < 10);
    nowMillis = millis();

    if(setdir == dir){
      //setpoint direction and current direction is the same
      flag = false;
      pid->update(abs(setpoint), speed, (nowMillis - prevMillis) / 1000.0, output);
      analogWrite(pinPWM, 255 - output);
    }

    if(setdir != dir){
      //not the same, need to brake first
      Serial.println(stop);

      if(!stop && !flag){
        flag = true;
        analogWrite(pinPWM, 255);
      }
      if(stop){
        pid->refresh();
        flag = false;
        dir = setdir;
        digitalWrite(pinDir, dir);
        // Serial.print(pinDir);
        // Serial.print(",");
        // Serial.println(dir);
      }
    }
    prevMillis = nowMillis;
  }

  void increment(){
	  if(lockinter) return;
	  timeStamp[idx ++] = micros();
	  idx = (idx & filtNum);
  }

  void updateSpeed(){
	  //have to lock the interrupt first
  	lockinter = 1;
  	
  	tt = (long)micros() - timeStamp[(idx + filtNum) & filtNum]; //current pulse width
  	tmp = (timeStamp[(idx + filtNum) & filtNum] - timeStamp[idx]) >> filtBitNum; //measured pulse width
  	
  	lockinter = 0;
      if(tt > 10000){
  		speed = speedRatio / tt;
       }else{
  		speed = (tmp == 0? 0 : speedRatio / tmp);
       }
       
  }

  void refresh(){
    //used when there is long time no moving
    pid->refresh();
    prevMillis = millis();
  }
};
#undef speedRatio
#undef filtNum
#undef filtBitNum

// define motor from pololu >>>>>>>>>>>>>>>>>>>>>>>>>>>>
#define speedRatio 0.0277777
class motorPololu : public motorGen{
public:
  int range[2] = {0, 255};
  int dBand[2] = {-35, 35};
  bool flag = 0, dir = 1, setdir, stop;
  int pinCh1, pinCh2, pinEna, pinIn1, pinIn2, whichMotor, output, N;
  float speed, interv;
  long prevMillis, nowMillis;
  long prevtime, now;
  PID *pid;
  //5, 10, 0.01
  

  motorPololu(int const &_pinCh1, int const &_pinCh2, int const &_pinEna, int const &_pinIn1, int const &_pinIn2):
  pinCh1(_pinCh1), pinCh2(_pinCh2), pinEna(_pinEna), pinIn1(_pinIn1), pinIn2(_pinIn2)
  {
    //add to the stack of motor pointers for use of glue isr;
	pinMode(pinEna, OUTPUT);
	pinMode(pinIn1, OUTPUT);
	pinMode(pinIn2, OUTPUT);
	pinMode(pinCh1, INPUT);
	pinMode(pinCh2, INPUT);
	digitalWrite(pinIn1, 1);
	digitalWrite(pinIn2, 0);
	analogWrite(pinEna, 0);
	
    whichMotor = motorCount;
    instance[motorCount++] = this;
    // Serial.print("The number of motor is ");
    // Serial.println(whichMotor);
    // pid = nullptr;
    speed = prevMillis = nowMillis = prevtime = now = 0;
  }

  void init(float Kp = 5 , float Ki = 10, float Kd = 0.01, int flags = 5, int *deadBand = nullptr){
  	N = 0;
    // Serial.println("Initialize motor PID controll");
    pid = new PID(Kp, Ki, Kd, flags, dBand, range);
    digitalWrite(pinIn1, 1); //current direction is 1
    digitalWrite(pinIn2, 0);
  	attachInterrupt(digitalPinToInterrupt(pinCh1), isr[whichMotor], CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinCh2), isr[whichMotor], CHANGE);
  }

  void drive(float const &setpoint){
    //flags
    // static 
    updateSpeed();
    setdir = (setpoint > 0);
    stop = (speed < 10);

    if(setdir == dir){
      //setpoint direction and current direction is the same
      flag = false;
      pid->update(abs(setpoint), speed, interv, output);
      analogWrite(pinEna, output);
    }

    if(setdir != dir){
      //not the same, need to brake first
      if(!stop && !flag){
        flag = true;
        analogWrite(pinEna, 0);
      }else if(stop){
        pid->refresh();
        flag = false;
        dir = setdir;
        digitalWrite(pinIn1, dir);
        digitalWrite(pinIn2, !dir);
      }
    }
  }

  void increment(){
    N++;
  }

  void updateSpeed(){
  	now = micros();
  	interv = (now - prevtime) / 1e6;
  	  //have to lock the interrupt first
      speed = speedRatio * N / interv;
      N = 0;
  	prevtime = now;
  }

  void refresh(){
    //used when there is long time no moving
	  N = 0;
    pid->refresh();
    prevtime = micros();
  }
};
#define speedRatio

