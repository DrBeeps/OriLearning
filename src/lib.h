#ifndef _LIB_H_
#define _LIB_H_

#include <Arduino.h>
#include <Servo.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

#include "BMI088.h"
#include "Orientation.h"
#include "gnc.cpp"

Servo servoX;
Servo servoY;

Bmi088Accel accel(Wire,0x19);
Bmi088Gyro gyro(Wire,0x69);

double dt;

double PIDX, PIDY;
float kp, ki, kd;

double p_errorX, p_errorY;
double errorX, errorY;

double Ax, Ay;
double d_angleX, d_angleY;
 
float pwmX, pwmY;

double pidX_p, pidX_i, pidX_d;
double pidY_p, pidY_i, pidY_d;

float servo_gear_ratio = 1;
float servo_off;


double PreviousGyroX, PreviousGyroY, PreviousGyroZ;
double IntGyroX, IntGyroY, IntGyroZ;

double OreX, OreY, OreZ;
double OrientationX, OrientationY, OrientationZ;

double DifGyroX, DifGyroY, DifGyroZ;

double matrix1, matrix2, matrix3, matrix4, matrix5, matrix6, matrix7, matrix8, matrix9;
double OutX, OutY;

int serialCounter = 0;

uint64_t lastMicros;
uint64_t currentMicros;

// Perry Lib Functions //

Orientation ori;
EulerAngles gyroData;
EulerAngles gyroOut;

// =================== //

int servo_homeX = 0;
int servo_homeY = 0;


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

void tvcTest()
{
    servoX.write(servo_homeX - 25);
    delay(500);
    servoX.write(servo_homeX)
    delay(500);
    servoY.write(servo_homeY - 25);
    delay(500);
    servoY.write(servo_homeY)
}
// ============= //


#endif