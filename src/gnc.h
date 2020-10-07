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

// =====    Rocket Orientation | Quaternions    ===== //

void stabilize(double dt) 
{
  gyro.readSensor();

  gyroData.roll = gyro.getGyroX_rads();
  gyroData.yaw = gyro.getGyroZ_rads();
  gyroData.pitch = gyro.getGyroY_rads();

  ori.update(gyroData, dt);
  gyroOut = ori.toEuler();
  
  LocalOrientationX = (gyroOut.roll * RAD_TO_DEG);
  LocalOrientationY = (gyroOut.pitch * RAD_TO_DEG);
  LocalOrientationZ = (gyroOut.yaw * RAD_TO_DEG);

  Serial.print("ORE Z => "); Serial.print(LocalOrientationZ); Serial.print("\n");
  Serial.print("ORE Y => "); Serial.print(LocalOrientationY); Serial.print("\n");

  // Serial.print("pitch integrated (deg) "); Serial.print(IntGyroY); Serial.print("\n");
  // Serial.print("yaw integrated (deg) "); Serial.print(IntGyroZ); Serial.print("\n");
  // Serial.print("roll integrated (deg) "); Serial.print(IntGyroX); Serial.print("\n");
  pwmZ = zAxis.update(LocalOrientationZ, dt);
  pwmY = yAxis.update(LocalOrientationY, dt);

  Serial.print("PWM Z => "); Serial.print(pwmZ); Serial.print("\n");
  Serial.print("PWM Y => "); Serial.print(pwmY); Serial.print("\n");

  servoZ.write(pwmZ);
  servoY.write(pwmY);
}

// ================================================== //

// ==============  Physics Simulations  ============= //





// ================================================== //

#endif 