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
void pid(double IntGyroY, double IntGyroZ, double dt) 
{
  p_errorZ = errorZ;
  p_errorY = errorY; 

  errorZ = d_angleZ - IntGyroZ;
  errorY = d_angleY - IntGyroY;

  pidZ_p = kp*errorZ;
  pidY_p = kp*errorY;

  pidZ_d = kd*((errorZ - p_errorZ)/dt);
  pidY_d = kd*((errorY - p_errorY)/dt);

  // pidZ_i = ki * (pidZ_i + errorZ * dt);
  // pidY_i = ki * (pidY_i + errorY * dt);

  PIDZ = pidZ_p + pidZ_d;
  PIDY = pidY_p + pidY_d;
  pwmZ = (PIDZ * SGR);
  pwmY = (PIDY * SGR);

  Serial.print("PWMZ => "); Serial.print(pwmZ); Serial.print("\n");
  Serial.print("PWMY => "); Serial.print(pwmY); Serial.print("\n");

  servoZ.write(pwmZ);
  servoY.write(pwmY);
} 
// ================================================== //

void pidTest2(double IntGyroZ, double IntGyroY, double dt)
{
  pErrorZ = errorZ;
  pErrorY = errorY;

  errorZ = (d_angleZ - IntGyroZ);
  errorY = (d_angleY - IntGyroY);

  errSumZ = errorZ;
  errSumY = errorY;

  dInputZ = (IntGyroZ - pIntGyroZ);

  PIDZ = (kp * errorZ) + (ki * errSumZ) + (kd * dInputZ);
  PIDY = (kp * errorY) + (ki * errSumY) + (kd * dInputY);

  pwmZ = (PIDZ * SGR);
  pwmY = (PIDY * SGR);
  
  capPwmVals(pwmZ, pwmY, 30);

  Serial.print(pwmZ); Serial.print(pwmY); Serial.print("\n");

  servoZ.write(pwmZ);
  servoY.write(pwmY);

  lastInputZ = dIntGyroZ;
  lastInputY = dIntGyroY;
}

class PID
{
  double computeServoVals(double LocalOrientation, double dAngle, double dt)
  {
    error = dAngle - LocalOrientation;

    errSum += (error * dt);

    compP = (kp * error);
    compI = (ki * errSum);
    compD = (error - lastError) / dt;

    lastError = error;

    pwm = compP + compI + compD;
    return pwm;
  }
  void writeServoVals(Servo servo, double pwmVal)
  {
    servo.write(pwmVal);
  }
};

void stabilize()
{
  pwmZ = zAxis.computeServoVals(LocalOrientationZ, 0, dt);
  pwmY = yAxis.computeServoVals(LocalOrientationY, 0, dt);

  zAxis.writeServoVals(servoZ, pwmZ);
  yAxis.writeServoVals(servoY, pwmY);
}


// ================================================== //
// =====    Rocket Orientation | Quaternions    ===== //

void getOrientation(double dt) 
{
  gyro.readSensor();

  gyroData.roll = gyro.getGyroX_rads();
  gyroData.yaw = -gyro.getGyroZ_rads();
  gyroData.pitch = -gyro.getGyroY_rads();

  ori.update(gyroData, dt);
  gyroOut = ori.toEuler();
  
  LocalOrientationX = (gyroOut.roll * RAD_TO_DEG);
  LocalOrientationY = (gyroOut.pitch * RAD_TO_DEG);
  LocalOrientationZ = (gyroOut.yaw * RAD_TO_DEG);

  // Serial.print("pitch integrated (deg) "); Serial.print(IntGyroY); Serial.print("\n");
  // Serial.print("yaw integrated (deg) "); Serial.print(IntGyroZ); Serial.print("\n");
  // Serial.print("roll integrated (deg) "); Serial.print(IntGyroX); Serial.print("\n");
  stabilize();
}

// ================================================== //

// ==============  Physics Simulations  ============= //





// ================================================== //

#endif 