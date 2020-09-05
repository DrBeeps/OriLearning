#ifndef _GNC_H_
#define _GNC_H_

#include "lib.h"
 
/*
TODO:
zero imu function
  needs to zero just before launch for proper inertial frame
  takes all gyro/accel readings and sets to zero
  possible ways of doing that:
    - soft reset of sensor:

nail down ypr DCM matrix evaluation
find out how the matrix responds to roll
FIX THE IMU ¯\_(ツ)_/¯

implemented trapezoidal cummulative integration!


VERY IMPORTANT:
REMEMBER TO ADD SERVO UPDATE CYCLE TO GNC AND PID 
OUTPUT OR THE FLIGHT PERFORMANCE WILL BE SHIT
*/

void servowrite()  
{
 servoY.write(pwmY);
 servoZ.write(pwmZ); 
}

void printYPR(EulerAngles out)
{
  Serial.print(out.yaw * RAD_TO_DEG);
  Serial.print("\t");
  Serial.print(out.pitch * RAD_TO_DEG);
  Serial.print("\t");
  Serial.print(out.roll * RAD_TO_DEG);
  Serial.print("\n");
}

// ================================================== //
// ===            PID Functions | Math            === //
void pid(double localOrientationY, double localOrientationZ, double dt)
{
  p_errorZ = errorZ;
  p_errorY = errorY; 

  errorZ = localOrientationZ - d_angleZ;
  errorY = localOrientationY - d_angleY;

  pidZ_p = kp*errorZ;
  pidY_p = kp*errorY; 

  pidZ_d = kd*((errorZ - p_errorZ)/dt);
  pidY_d = kd*((errorY - p_errorY)/dt);

  pidZ_i = ki * (pidZ_i + errorZ * dt);
  pidY_i = ki * (pidY_i + errorY * dt);

  PIDZ = pidZ_p + pidZ_i + pidZ_d;
  PIDY = pidY_p + pidY_i + pidY_d;

  pwmZ = (PIDZ * servo_gear_ratio); // calculate servo gear ratio ASAP
  pwmY = (PIDY * servo_gear_ratio);
  servoZ.write(pwmZ);
  servoY.write(pwmY);
} 
// ================================================== //


// ================================================== //
// ===    Rocket Orientation | Rotation Matrix    === //
void stabilize(EulerAngles gyroData, double dt)
{
  PreviousGyroX = IntGyroXrad;
  PreviousGyroY = IntGyroYrad;
  PreviousGyroZ = IntGyroZrad;

  Serial.print("pitch rads "); Serial.print(gyroData.pitch); Serial.print("\n");
  Serial.print("yaw rads "); Serial.print(gyroData.yaw); Serial.print("\n");
  Serial.print("roll rads "); Serial.print(gyroData.roll); Serial.print("\n");

  IntGyroX = gyroData.roll * dt;
  IntGyroY = gyroData.pitch * dt;
  IntGyroZ = gyroData.yaw * dt;

  Serial.print("pitch integrated (deg) "); Serial.print(IntGyroY); Serial.print("\n");
  Serial.print("yaw integrated (deg) "); Serial.print(IntGyroZ); Serial.print("\n");
  Serial.print("roll integrated (deg) "); Serial.print(IntGyroX); Serial.print("\n");

  IntGyroXrad = IntGyroX * (pi / 180);
  IntGyroYrad = IntGyroY * (pi / 180);
  IntGyroZrad = IntGyroZ * (pi / 180);

  Serial.print("pitch integrated (rads) "); Serial.print(IntGyroYrad); Serial.print("\n");
  Serial.print("yaw integrated (rads) "); Serial.print(IntGyroZrad); Serial.print("\n");
  Serial.print("roll integrated (rads) "); Serial.print(IntGyroXrad); Serial.print("\n");

  DifIntGyroX = (IntGyroXrad - PreviousGyroX);
  DifIntGyroY = (IntGyroYrad - PreviousGyroY);
  DifIntGyroZ = (IntGyroZrad - PreviousGyroZ);

  OreX = OrientationX;
  OreY = OrientationY;
  OreZ = OrientationZ;

  matrix1 = (cos(DifIntGyroX) * cos(DifIntGyroY));
  matrix2 = (((sin(DifIntGyroX) * -1) * cos(DifIntGyroZ) + (cos(DifIntGyroX)) * sin(DifIntGyroY) * sin(DifIntGyroZ)));
  matrix3 = ((sin(DifIntGyroX) * sin(DifIntGyroZ) + (cos(DifIntGyroX)) * sin(DifIntGyroY) * cos(DifIntGyroZ)));
  matrix4 = sin(DifIntGyroX) * cos(DifIntGyroY);
  matrix5 = ((cos(DifIntGyroX) * cos(DifIntGyroZ) + (sin(DifIntGyroX)) * sin(DifIntGyroY) * sin(DifIntGyroZ)));
  matrix6 = (((cos(DifIntGyroX) * -1) * sin(DifIntGyroZ) + (sin(DifIntGyroX)) * sin(DifIntGyroY) * cos(DifIntGyroZ)));
  matrix7 = (sin(DifIntGyroY)) * -1;
  matrix8 = cos(DifIntGyroY) * sin(DifIntGyroZ);
  matrix9 = cos(DifIntGyroY) * cos(DifIntGyroZ);

  OrientationZ = ((OreX * (matrix1)) + (OreY * (matrix2)) + (OreZ * (matrix3)));
  OrientationY = ((OreX * (matrix4)) + (OreY * (matrix5)) + (OreZ * (matrix6)));
  OrientationX = ((OreX * (matrix7)) + (OreY * (matrix8)) + (OreZ * (matrix9)));

  prev_localOrientationY = localOrientationY;
  prev_localOrientationZ = localOrientationZ;

  localOrientationZ = asin(OrientationZ) * (180 / pi) - differenceOreZ;
  localOrientationY = asin(OrientationX) * (180 / pi) - differenceOreY;

  differenceOreY = prev_localOrientationY - localOrientationY;
  differenceOreZ = prev_localOrientationZ - localOrientationZ;

  Serial.print("localorientation Z "); Serial.print(localOrientationZ); Serial.print("\n");
  Serial.print("localorientation Y "); Serial.print(localOrientationY); Serial.print("\n");
}
// ================================================== //

void zeroIMU()
{

}

#endif 