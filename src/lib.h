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

Servo servoY;
Servo servoZ;

Bmi088Accel accel(Wire,0x19);
Bmi088Gyro gyro(Wire,0x69);

double dt;

double PIDY, PIDZ;
float kp = 0.5;
float ki = 0.01;
float kd = 0.3;

double p_errorY, p_errorZ;
double errorZ, errorY;

double d_angleY = 0;
double d_angleZ = 0;
 
float pwmY, pwmZ;

double pidY_p, pidY_i, pidY_d;
double pidZ_p, pidZ_i, pidZ_d;

float servo_gear_ratio = 1;
float servo_off;


double PreviousGyroX, PreviousGyroY, PreviousGyroZ;
double IntGyroX, IntGyroY, IntGyroZ;
double IntGyroXrad, IntGyroYrad, IntGyroZrad;

double OreZ, OreY;
double OreX = 1;
double OrientationX = 1;
double OrientationY, OrientationZ;

double DifIntGyroX, DifIntGyroY, DifIntGyroZ;

double matrix1, matrix2, matrix3, matrix4, matrix5, matrix6, matrix7, matrix8, matrix9;
double OutX, OutZ;

double Z_IntGyroX, Z_IntGyroY, Z_IntGyroZ, prev_IntGyroX, prev_IntGyroY, prev_IntGyroZ;
double av_IntGyroX, av_IntGyroY, av_IntGyroZ;
double prev_IntGyroX2, prev_IntGyroY2, prev_IntGyroZ2;
double ZeroedX, ZeroedY, ZeroedZ, Z_T;

double orientationCordY, orientationCordZ;

double differenceOreZ, differenceOreY;
double prev_localOrientationZ, prev_localOrientationY;
double localOrientationZ, localOrientationY;
int serialCounter = 0;

uint64_t lastMicros;
uint64_t currentMicros;

// Perry Lib Functions //

Orientation ori;
EulerAngles gyroData;
EulerAngles gyroOut;

// =================== //

int servo_homeZ = 20;
int servo_homeY = 20;


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
    servoZ.write(servo_homeZ);
    servoY.write(servo_homeY);
}


void servo_test()
{
  servoZ.write(servo_homeZ - 20);
  delay(120);
  servoZ.write(servo_homeZ);
  delay(120);
  servoY.write(servo_homeY - 20);
  delay(120);
  servoY.write(servo_homeY);
  delay(120);
  servoZ.write(servo_homeZ + 20);
  delay(120);
  servoZ.write(servo_homeZ);
  delay(120);
  servoY.write(servo_homeY + 20);
  delay(120);
  servoY.write(servo_homeY);
  delay(120);
  Serial.println("SERVO TEST COMPLETE");
  delay(300);
}
// Designed an entire new TVC Mount! TVCV2 for use in Styfe

// ============= //

double gpI;
double gyI;

// cumErrorX += errorX * elapsedTime;

#endif