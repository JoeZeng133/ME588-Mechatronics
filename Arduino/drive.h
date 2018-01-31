#include "utils.h"
#include "comm.h"

# define N 3

class drivetrain {
  const float pi = 3.14159265;
  const float sang[N] = {sin(pi), sin(-pi/3), sin(pi/3)}; // Sine of angles from the center to the wheels CCW from forward
  const float cang[N] = {cos(pi), cos(-pi/3), cos(pi/3)}; // Sine of angles from the center to the wheels CCW from forward
  const float R = 19; // Use whatever units you want the speed in for these
  const float Rw = 3.15;
  const float maxRPM = 100;
  float speed[3];
  byte driveMode = 0;
  float referenceHeading;
  float prev_cmd[4];
  
  public:
     motorPololu* mot1; 
     motorDigikey* mot[N-1];
     void initAll();
     void refreshAll();
     bool newCommand();
     void setReference();
     void updateDrive();
     void setSpeeds(float* speeds);
     void relativeHeadingPolar(float V, float phi, float omega);
     void relativeHeadingXYR(float Vx, float Vy, float omega);
     void absoluteHeadingPolar(float V, float phi, float omega);
};

void drivetrain::initAll() {
  mot1 -> init();
  for (int i = 1; i < N; i++) {
    mot[i-1] -> init();
  }
}

void drivetrain::refreshAll() {
  mot1->refresh();
  for (int i = 1; i < N; i++) {
    mot[i-1] -> refresh();
  }
}

bool drivetrain::newCommand() {
  return (command.drive_cmd != prev_cmd[0])||(command.drive_data[0] != prev_cmd[1])||(command.drive_data[1] != prev_cmd[2])||(command.drive_data[2] != prev_cmd[3]);
}

void drivetrain::setReference() {
  referenceHeading = state.ypr[0];
}

void drivetrain::updateDrive() {
  switch(command.drive_cmd) {
    case 0: // do nothing
      break;
    case 1: // stop
      mot1->pid->refresh();
      // mot1->stop();
      mot[0]->stop();mot[0]->pid->refresh();
      mot[1]->stop();mot[1]->pid->refresh();
      for(int i = 0;i < N; ++i) speed[i] = 0;
        setSpeeds(speed);


      break;
    case 2: // Relative motion in polar coordinates
        relativeHeadingPolar(command.drive_data[0], command.drive_data[1], command.drive_data[2]);
      break;
    case 3: // Relative motion in cartesian coordinates
        relativeHeadingXYR(command.drive_data[0], command.drive_data[1], command.drive_data[2]);
      break;
    case 4: // Absolute motion in polar coordinates
      absoluteHeadingPolar(command.drive_data[0], command.drive_data[1], command.drive_data[2]);
      break;
  }

  state.drive_mode = command.drive_cmd;
  // if (newCommand()) {
  //   prev_cmd[0] = command.drive_cmd;
  //   prev_cmd[1] = command.drive_data[0];
  //   prev_cmd[2] = command.drive_data[1];
  //   prev_cmd[3] = command.drive_data[2];
  //   refreshAll();
  // }
}

void drivetrain::setSpeeds(float* speeds) {

  // mot[0] -> drive(80);
  // mot[1] -> drive(0);
    mot1 -> drive(speeds[0]);
  for (int i = 1; i < 3; i++) {
    mot[i-1] -> drive(speeds[i]);
  }
  // Serial.println(' ');
  state.drive_rpm[0] = 200;
    // for (int i = 1; i < 3; i++) {
    //     state.drive_rpm[i] = mot[i - 1] -> output;
    // }
}

void drivetrain::relativeHeadingPolar(float V, float phi, float omega) {
  //convert to XYR and pass to the correct function
  relativeHeadingXYR(V*(float)cos(phi*pi/180), V*(float)sin(phi*pi/180), omega);
//    Serial.print(V);
//    Serial.print(' ');
//    Serial.println(phi);
}

void drivetrain::relativeHeadingXYR(float Vx, float Vy, float omega) {
  float speeds[N];

  // Calculate necessary wheel speeds in RPM
  speeds[0] = (-Vx*sang[0]+Vy*cang[0]+R*omega)*9.549/Rw;
  for (int i = 1; i < N; i++) {
    speeds[i] = (-Vx*sang[i]+Vy*cang[i]+R*omega)*9.549/Rw;
  }

  // Check that none are above the maximum and limit to preserve the 
  // correct motion if the speed is unattainable
  float M = 0;
  
  for (int i = 0; i < N; i++) {
    M = max(M,abs(speeds[i]));
  }
  
  if (M > maxRPM) {
    //Serial.println("Warning: Maximum speed exceeded. Throttling to retain heading");
    float s = abs(maxRPM/M);
    for (int i = 0; i < N; i++) {
      speeds[i] = s*speeds[i];
    }
  }

  // Set the wheel speeds
  setSpeeds(speeds);

  // Serial.print(mot1->speed_motor);
  // Serial.print(" ");
  // Serial.print(mot[0]->speed_motor);
  // Serial.print(" ");
  // Serial.print(mot[1]->speed_motor);
  // Serial.print(" ");

 // for(int i = 0; i < N; i++) {
 //   Serial.print(speeds[i]);
 //   Serial.print(" ");
 // }
 // Serial.print("\n");
}

void drivetrain::absoluteHeadingPolar(float V, float phi, float omega) {
  relativeHeadingPolar(V,phi-referenceHeading-state.ypr[0],omega);
}

