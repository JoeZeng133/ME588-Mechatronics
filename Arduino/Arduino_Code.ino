#include <SPI.h>

#include "comm.h"
#include "drive.h"
#include "ultrasonic.h"
#include "switches.h"
#include "I2Cdev/I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include <Servo.h>

unsigned long t;
unsigned long prevtime_drv = 0;
unsigned long prevtime_mot = 0;
unsigned long prevtime_srv = 0;
unsigned long prevtime_com = 0;
unsigned long prevtime_mpu = 0;
unsigned long prevtime_us = 0;
unsigned long prevtime_sw = 0;

const unsigned long update_time_drv = 10;
const unsigned long update_time_mot = 10;
const unsigned long update_time_srv = 10;
const unsigned long update_time_com = 50;
const unsigned long update_time_mpu = 5;
const unsigned long update_time_us = 100;
const unsigned long update_time_sw = 10;

// Drive Parameters
int pinCh1 = 19, pinCh2 = 999;
int pinEna = 8;
int pinIn1 = 23, pinIn2 = 25;
int setpoint = 70;
int dead[2] = { -35, 35};
int pinPWM[2] = {3, 5};
int pinEnc[2] = {2, 18};
int pinDir[2] = {15, 16};
drivetrain drive3;

// Secondary motor parameters
int RollerEn[N_mot] = {10};
int RollerIn1[N_mot] = {31};
int RollerIn2[N_mot] = {33};
HBridgeMotor motList[N_mot];


// MPU parameters
MPU6050 mpu;  //MPU6050 mpu(0x69); // <-- use for AD0 high
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];         // [psi, theta, phi]    Euler angle container


// servo parameters
Servo servoList[N_servo];
int servoPins[N_servo] = {12, 13};

// switch parameters
LimitSwitch switchList[N_lim];
int switchPinList[N_lim] = {41, 43, 45, 47, 49};

// ultrasonic parameters
// ultrasonic usList[N_us];
int usEch[N_us] = {3};
int usTrg[N_us] = {4};

void setup() {
  Serial.begin(115200);

  // set up mpu
  // #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  //     Wire.begin();
  //     Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  // #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  //     Fastwire::setup(400, true);
  // #endif
  // mpu.initialize();
  // mpu.dmpInitialize();
  // mpu.setDMPEnabled(true);
  // packetSize = mpu.dmpGetFIFOPacketSize();

  //set up drive
  motorGen::begin();
  drive3.mot1 = new motorPololu(pinCh1, pinCh2, pinEna, pinIn1, pinIn2);
  drive3.mot[0] = new motorDigikey(pinEnc[0], pinPWM[0], pinDir[0]);
  drive3.mot[1] = new motorDigikey(pinEnc[1], pinPWM[1], pinDir[1]);
  drive3.initAll();

  // set up secondary motors
  for (int i = 0; i < N_mot; i++) {
    motList[i].begin_mot(RollerEn[i], RollerIn1[i], RollerIn2[i]);
  }

  command.mot_cmd[0] = 0;
  command.mot_cmd[1] = 0;

  // set up servos
  for (int i = 0; i < N_servo; i++) {
    servoList[i] = Servo();
    servoList[i].attach(servoPins[i]);
  }
  
  // set up limit switches
  for (int i = 0; i < N_lim; i++) {
    switchList[i].begin_sw(switchPinList[i],i);
  }

  // set up ultrasonic
  // for (int i = 0; i < N_us; i++) {
  //   usList[i].begin_us(i,usTrg[i],usEch[i]);
  // }

  command.drive_cmd = 1;
  command.drive_data[0] = 0;
  command.drive_data[1] = 0;
  command.servo_cmd[0] = 90;
  command.servo_cmd[1] = 90;
 // Serial.println(sizeof(state));
}

long testprev = 0, tmpstate = 0, testidx = 0;
float testSpeed[8] = {-2, -1.5, -0.5, 0.1, 0.3, 0.6, 0.9, 1.5};



void loop() {
  
t = millis();

// if (t - testprev > 2000){
   
//   command.drive_data[0] = 0;
//   command.drive_data[1] = 0;
//   command.drive_cmd = (tmpstate ? 3 : 1);
//   // command.drive_data[2] = testSpeed[testidx];
//   command.drive_data[2] = 1;
//   testidx = testidx % 8;
//   command.servo_cmd[0] = 0;
//   tmpstate = !tmpstate;
//   // Serial.println(command.drive_data)
//   // Serial.println(command.drive_data[2]);
//   testprev = t;
// }
 

//   update comms
  if (t - prevtime_com > update_time_com) {
    if (read_command(command)) {
      state.drive_mode = command.servo_cmd[0]+command.servo_cmd[1];
      write_status(state);
    }
    prevtime_com = t;
  }

  
  // update mpu reading
  // if (t - prevtime_mpu > update_time_mpu) {
  //   fifoCount = mpu.getFIFOCount();
  //   mpuIntStatus = mpu.getIntStatus();
    
  //   if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
  //       mpu.resetFIFO();
  //   } 
  //   else if (mpuIntStatus & 0x02) {
  //       // wait for correct available data length, should be a VERY short wait
  //       while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

  //       // read a packet from FIFO
  //       mpu.getFIFOBytes(fifoBuffer, packetSize);
  //       fifoCount -= packetSize;

  //       // display Euler angles in degrees
  //       mpu.dmpGetQuaternion(&q, fifoBuffer);
  //       mpu.dmpGetGravity(&gravity, &q);
  //       mpu.dmpGetYawPitchRoll(state.ypr, &q, &gravity);
  //       state.ypr[0] = state.ypr[0]*-180/3.14159;
  //       state.ypr[1] = state.ypr[1]*180/3.14159;
  //       state.ypr[2] = state.ypr[2]*180/3.14159;
  //   }
  //   prevtime_mpu = t;
  // }


  // update drive
  if (t - prevtime_drv > update_time_drv) {
    drive3.updateDrive();
    prevtime_drv = t;
  }

  // update servos
  if (t - prevtime_srv > update_time_srv) {
    for(int i = 0; i < N_servo; i++) {
      servoList[i].write(command.servo_cmd[i]);
    }
    prevtime_srv = t;
  }

  // update secondary motors
  if (t - prevtime_mot > update_time_mot) {
    for(int i = 0; i < N_mot; i++) {
      motList[i].drive(command.mot_cmd[i]);
    }
    prevtime_mot = t;
  }

  // update limit switches
  if (t - prevtime_sw > update_time_sw) {
    for(int i = 0; i < N_lim; i++) {
      switchList[i].update_sw();
    }
    prevtime_sw = t;
  }

  // update ultrasonic
  // if (t - prevtime_us > update_time_us) {
  //   for(int i = 0; i < N_mot; i++) {
  //     if (command.US_enable) {
  //       usList[i].ping();
  //     }
  //   }
  //   prevtime_us = t;
  // }
}
