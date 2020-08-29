#ifndef _LIB_H_
#define _LIB_H_
 
#include <Arduino.h>
#include <Servo.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

#include "BMI088.h"
#include "Orientation.h"
 
#define pi 3.14159265358979

Servo servoX;
Servo servoZ;

Bmi088Accel accel(Wire,0x19);
Bmi088Gyro gyro(Wire,0x69);

double dt;

double PIDX, PIDZ;
float kp, ki, kd;

double p_errorX, p_errorZ;
double errorX, errorZ;

double Ax, Az;
double d_angleX, d_angleZ;
 
float pwmX, pwmZ;

double pidX_p, pidX_i, pidX_d;
double pidZ_p, pidZ_i, pidZ_d;

float servo_gear_ratio = 1;
float servo_off;


double PreviousGyroX, PreviousGyroY, PreviousGyroZ;
double IntGyroX, IntGyroY, IntGyroZ;

double OreX, OreY, OreZ;
double OrientationX, OrientationY, OrientationZ;

double DifGyroX, DifGyroY, DifGyroZ;

double matrix1, matrix2, matrix3, matrix4, matrix5, matrix6, matrix7, matrix8, matrix9;
double OutX, OutZ;

double Z_IntGyroX, Z_IntGyroY, Z_IntGyroZ, prev_IntGyroX, prev_IntGyroY, prev_IntGyroZ;
double av_IntGyroX, av_IntGyroY, av_IntGyroZ;
double prev_IntGyroX2, prev_IntGyroY2, prev_IntGyroZ2;
double ZeroedX, ZeroedY, ZeroedZ, Z_T;

double localOrientationZ, localOrientationY;
double orientationCordY, orientationCordZ;

int serialCounter = 0;

uint64_t lastMicros;
uint64_t currentMicros;

// Perry Lib Functions //

Orientation ori;
EulerAngles gyroData;
EulerAngles gyroOut;

// =================== //

int servo_homeX = 20;
int servo_homeZ = 20;


// OOP Functions //

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


void servoHome()
{
    servoX.write(servo_homeX);
    servoZ.write(servo_homeZ);
}


void tvcTest()
{
    servoX.write(servo_homeX - 20);
    delay(140);
    servoX.write(servo_homeX);
    delay(140);
    servoZ.write(servo_homeZ - 20);
    delay(140);
    servoZ.write(servo_homeZ);
    delay(140);
    servoX.write(servo_homeX + 20);
    delay(140);
    servoX.write(servo_homeX);
    delay(140);
    servoZ.write(servo_homeZ + 20);
    delay(140);
    servoZ.write(servo_homeZ);
    delay(500);
}
// Designed an entire new TVC Mount! TVCV2 for use in Styfe

// ============= //

double computeIntegrateX(double _GyroN, double dt)
{
    double ret += _GyroN * dt;
    return ret;
}


#endif