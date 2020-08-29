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
 servoX.write(pwmX);
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
void pid(double Ax, double Az, double dt)
{
  p_errorX = errorX;
  p_errorZ = errorZ; 

  errorX = Ax - d_angleX;
  errorZ = Az - d_angleZ;

  pidX_p = kp*errorX;
  pidZ_p = kp*errorZ; 

  pidX_d = kd*((errorX - p_errorX)/dt);
  pidZ_d = kd*((errorZ - p_errorZ)/dt);

  pidX_i = ki * (pidX_i + errorX * dt);
  pidZ_i = ki * (pidZ_i + errorZ * dt);

  PIDX = pidX_p + pidX_i + pidX_d;
  PIDZ = pidZ_p + pidZ_i + pidZ_d;

  pwmX = (PIDX * servo_gear_ratio); // calculate servo gear ratio ASAP
  pwmZ = (PIDZ * servo_gear_ratio);
  servoX.write(pwmX);
  servoZ.write(pwmZ);
} 
// ================================================== //


// ================================================== //
// ===    Rocket Orientation | Rotation Matrix    === //
void stabilize(EulerAngles gyroData, double dt)
{
  gyro.readSensor();
  Serial.println(gyro.getGyroX_rads());
  Serial.println(gyro.getGyroY_rads());
  Serial.println(gyro.getGyroZ_rads());
  
  gyroData.pitch = -gyro.getGyroY_rads();
  gyroData.yaw = -gyro.getGyroZ_rads();
  gyroData.roll = gyro.getGyroX_rads();   

  ori.update(gyroData, dt);
  gyroOut = ori.toEuler();
 
  PreviousGyroX = IntGyroX;
  PreviousGyroY = IntGyroY;
  PreviousGyroZ = IntGyroZ;


  // angle += rate * dt
  IntGyroY = gyroOut.yaw * RAD_TO_DEG;
  IntGyroZ = gyroOut.pitch * RAD_TO_DEG;
  IntGyroX = gyroOut.roll * RAD_TO_DEG;

  OreX = OrientationX;
  OreY = OrientationY;
  OreZ = OrientationZ;

  DifGyroX = (IntGyroX - PreviousGyroX);
  DifGyroY = (IntGyroY - PreviousGyroY);
  DifGyroZ = (IntGyroZ - PreviousGyroZ);

  //X Matrices
  matrix1 = (cos(DifGyroZ) * cos(DifGyroY));
  matrix2 = (((sin(DifGyroZ) * -1) * cos(DifGyroX) + (cos(DifGyroZ)) * sin(DifGyroY) * sin(DifGyroX)));
  matrix3 = ((sin(DifGyroZ) * sin(DifGyroX) + (cos(DifGyroZ)) * sin(DifGyroY) * cos(DifGyroX)));
  
  //Y Matrices
  matrix4 = sin(DifGyroZ) * cos(DifGyroY);
  matrix5 = ((cos(DifGyroZ) * cos(DifGyroX) + (sin(DifGyroZ)) * sin(DifGyroY) * sin(DifGyroX)));
  matrix6 = (((cos(DifGyroZ) * -1) * sin(DifGyroX) + (sin(DifGyroZ)) * sin(DifGyroY) * cos(DifGyroX)));
  
  //Z Matrices
  matrix7 = (sin(DifGyroY)) * -1;
  matrix8 = cos(DifGyroY) * sin(DifGyroX);
  matrix9 = cos(DifGyroY) * cos(DifGyroX);
  
  OrientationX = ((OreX * matrix1)) + ((OreZ * matrix2)) + ((OreZ * matrix3));
  OrientationY = ((OreX * matrix4)) + ((OreZ * matrix5)) + ((OreZ * matrix6));
  OrientationZ = ((OreX * matrix7)) + ((OreZ * matrix8)) + ((OreZ * matrix9));

  localOrientationZ = asin(OrientationY) * (-180 / pi) - differenceOreZ;
  localOrientationY = asin(OrientationZ) * (180 / pi) - differenceOreY;

  Serial.print("pOZ => "); Serial.print(localOrientationZ); Serial.print("\t");
  Serial.print("pOY => "); Serial.print(localOrientationY); Serial.print("\n");

  pid(localOrientationZ, localOrientationY, dt);
  delay(100);
}
// ================================================== //


void trapezoidalCummulativeIntegration(EulerAngles gyroData, double dt)
{
  gyroData.pitch = -gyro.getGyroY_rads();
  gyroData.yaw = -gyro.getGyroZ_rads();
  gyroData.roll = gyro.getGyroX_rads();

  prev_IntGyroX = gyroData.roll * RAD_TO_DEG;
  prev_IntGyroY = gyroData.pitch * RAD_TO_DEG;
  prev_IntGyroZ = gyroData.yaw * RAD_TO_DEG;
  // this is not correct... fix later
  ori.update(gyroData, dt);
  gyroOut = ori.toEuler();

  prev_IntGyroX2 = gyroOut.roll * RAD_TO_DEG;
  prev_IntGyroY2 = gyroOut.pitch * RAD_TO_DEG;
  prev_IntGyroZ2 = gyroOut.yaw * RAD_TO_DEG;

  av_IntGyroX = (prev_IntGyroX + prev_IntGyroX2) / 2;
  av_IntGyroY = (prev_IntGyroY + prev_IntGyroY2) / 2;
  av_IntGyroZ = (prev_IntGyroZ + prev_IntGyroZ2) / 2;
}

void zeroIMU()
{

}

#endif 