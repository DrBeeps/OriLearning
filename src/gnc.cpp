#include "includes.h"
 
/*
TODO:
Fix PID odd values
properly calibrate imu

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

  gyroData.roll = (gyro.getGyroX_rads() - IMUOffsetX);
  gyroData.pitch = (-gyro.getGyroZ_rads() - IMUOffsetZ);
  gyroData.yaw = (-gyro.getGyroY_rads() - IMUOffsetY);

  ori.update(gyroData, dt);
  gyroOut = ori.toEuler();
  
  LocalOrientationX = (gyroOut.roll * RAD_TO_DEG);
  LocalOrientationY = (gyroOut.pitch * RAD_TO_DEG);
  LocalOrientationZ = (gyroOut.yaw * RAD_TO_DEG);

  Serial.print("ORE Z => "); Serial.print(LocalOrientationZ); Serial.print("\n");
  Serial.print("ORE Y => "); Serial.print(LocalOrientationY); Serial.print("\n");

  pwmZ = zAxis.update(LocalOrientationZ, dt);
  pwmY = yAxis.update(LocalOrientationY, dt);

  cs = cos(-LocalOrientationX);
  sn = sin(-LocalOrientationX);
  trueYOut = pwmY * cs - pwmZ * sn;
  trueZOut = pwmY * sn + pwmZ * cs;

  Serial.print("PWM Z => "); Serial.print(trueYOut); Serial.print("\n");
  Serial.print("PWM Y => "); Serial.print(trueZOut); Serial.print("\n");

  servoZ.write(trueZOut);
  servoY.write(trueYOut);
}

// ================================================== //
