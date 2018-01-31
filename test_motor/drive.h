#include "utils.h"

# define N 3

class drivetrain {
  const float pi = 3.14159265;
  const float sang[N] = {sin(pi), sin(-pi/3), sin(pi/3)}; // Sine of angles from the center to the wheels CCW from forward
  const float cang[N] = {cos(pi), cos(-pi/3), cos(pi/3)}; // Sine of angles from the center to the wheels CCW from forward
  const float R = 19; // Use whatever units you want the speed in for these
  const float Rw = 3.15;
  const float maxRPM = 100;
  
  public:
     motorPololu* mot1; 
     motorDigikey* mot[N-1];
     void initAll();
     void refreshAll();
     void setSpeeds(float* speeds);
     void relativeHeadingPolar(float V, float phi, float omega);
     void relativeHeadingXYR(float V, float phi, float omega);
    
};

void drivetrain::initAll() {
  mot1 -> init();
  for (int i = 1; i < N; i++) {
    mot[i-1] -> init();
  }
}

void drivetrain::refreshAll() {
  mot1 -> refresh();
  for (int i = 1; i < N; i++) {
    mot[i-1] -> refresh();
  }
}

void drivetrain::setSpeeds(float* speeds) {
  mot1 -> drive(speeds[0]);
  for (int i = 1; i < N; i++) {
    mot[i-1] -> drive(speeds[i]);
  }
}

void drivetrain::relativeHeadingPolar(float V, float phi, float omega) {
  //convert to XYR and pass to the correct function
  relativeHeadingXYR(V*(float)cos(phi*pi/180), V*(float)sin(phi*pi/180), omega);
}

void drivetrain::relativeHeadingXYR(float Vx, float Vy, float omega) {
  float speeds[N];

  // Calculate necessary wheel speeds in RPM
  speeds[0] = (-Vx*sang[0]+Vy*cang[0]+R*omega)*9.549/Rw;
  for (int i = 1; i < N; i++) {
    speeds[i] = (-Vx*sang[i]+Vy*cang[i]+R*omega)*9.549/Rw;
  }
    Serial.println((60/(2*pi)));

  // Check that none are above the maximum and limit to preserve the 
  // correct motion if the speed is unattainable
  float M = 0;
  
  for (int i = 0; i < N; i++) {
    M = max(M,abs(speeds[i]));
  }
  
  if (M > maxRPM) {
    Serial.println("Warning: Maximum speed exceeded. Throttling to retain heading");
    float s = abs(maxRPM/M);
    for (int i = 1; i < N; i++) {
      speeds[i] = s*speeds[i];
    }
  }

  // Set the wheel speeds
  setSpeeds(speeds);
}
