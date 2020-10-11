#include "includes.h"

void setupIMU()
 {
    int status;
  
    Serial.begin(115200);

    while(!Serial) {}

    status = accel.begin();

    if (status < 0) 
    {
        Serial.println("Accel Initialization Error");
        Serial.println(status);
        while (1) {}
    }
    status = gyro.begin();
    if (status < 0) 
    {
        Serial.println("Gyro Initialization Error");
        Serial.println(status);
        while (1) {}
    }
    while(!gyro.getDrdyStatus()) {}
}

void zeroIMURaw()
{
    for(IMUSampleTime = 0; IMUSampleTime < 10; IMUSampleTime++)
    {
        gyro.readSensor();

        IMUValX += gyro.getGyroX_rads();
        IMUValY += gyro.getGyroY_rads();
        IMUValZ += gyro.getGyroZ_rads();
    }
    IMUOffsetX = IMUValX / 10;
    IMUOffsetY = IMUValY / 10;
    IMUOffsetZ = IMUValZ / 10;

    Serial.print("Offset X"); Serial.print(IMUOffsetX); Serial.print("\t");
    Serial.print("Offset Y"); Serial.print(IMUOffsetY); Serial.print("\t");
    Serial.print("Offset Z"); Serial.print(IMUOffsetZ); Serial.print("\t");
}

void getOrientation(double dt)
{
  gyro.readSensor();

  gyroData.roll = (gyro.getGyroX_rads() - abs(IMUOffsetX));
  gyroData.pitch = (-gyro.getGyroZ_rads() - abs(IMUOffsetZ));
  gyroData.yaw = (-gyro.getGyroY_rads() - abs(IMUOffsetY));

  ori.update(gyroData, dt);
  gyroOut = ori.toEuler();
  
  LocalOrientationX = (gyroOut.roll * RAD_TO_DEG);
  LocalOrientationY = (gyroOut.pitch * RAD_TO_DEG);
  LocalOrientationZ = (gyroOut.yaw * RAD_TO_DEG);
}

void zeroIMUDeg(double dt)
{
    for(IMUSampleTime = 0; IMUSampleTime < 10; IMUSampleTime++)
    {
        getOrientation(dt);

        IMUValX += LocalOrientationX;
        IMUValY += LocalOrientationY;
        IMUValZ += LocalOrientationZ;
    }
    IMUOffsetX = IMUValX / 10;
    IMUOffsetY = IMUValY / 10;
    IMUOffsetZ = IMUValZ / 10;

    Serial.print("Offset X"); Serial.print(IMUOffsetX); Serial.print("\t");
    Serial.print("Offset Y"); Serial.print(IMUOffsetY); Serial.print("\t");
    Serial.print("Offset Z"); Serial.print(IMUOffsetZ); Serial.print("\t");
}