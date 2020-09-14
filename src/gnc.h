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

void servowrite() {
 servoY.write(pwmY);
 servoZ.write(pwmZ); 
}

void printYPR(EulerAngles out) {
  Serial.print(out.yaw * RAD_TO_DEG);
  Serial.print("\t");
  Serial.print(out.pitch * RAD_TO_DEG);
  Serial.print("\t");
  Serial.print(out.roll * RAD_TO_DEG);
  Serial.print("\n");
}

// ================================================== //
// ===            PID Functions | Math            === //
void pid(double IntGyroY, double IntGyroZ, double dt) {
  p_errorZ = errorZ;
  p_errorY = errorY; 

  errorZ = IntGyroZ - d_angleZ;
  errorY = IntGyroY - d_angleY;

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
// =====    Rocket Orientation | Quaternions    ===== //

void intTest(double dt) {
  gyro.readSensor();

  gyroData.roll = gyro.getGyroX_rads();
  gyroData.yaw = -gyro.getGyroZ_rads();
  gyroData.pitch = -gyro.getGyroY_rads();

  ori.update(gyroData, dt);
  gyroOut = ori.toEuler();
  
  IntGyroX = (gyroOut.roll * RAD_TO_DEG);
  IntGyroY = (gyroOut.pitch * RAD_TO_DEG);
  IntGyroZ = (gyroOut.yaw * RAD_TO_DEG);

  Serial.print("pitch integrated (deg) "); Serial.print(IntGyroY); Serial.print("\n");
  Serial.print("yaw integrated (deg) "); Serial.print(IntGyroZ); Serial.print("\n");
  Serial.print("roll integrated (deg) "); Serial.print(IntGyroX); Serial.print("\n");
  pid(IntGyroY, IntGyroZ, dt);
}

// ================================================== //

#endif 