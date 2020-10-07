#ifndef _LIB_H_
#define _LIB_H_

#include <vector>

#include <Arduino.h>
#include <Servo.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

#include "BMI088.h"
#include "Orientation.h"
#include <vector>
#define pi 3.14159265358979

Servo servoY;
Servo servoZ;

Bmi088Accel accel(Wire,0x19);
Bmi088Gyro gyro(Wire,0x69);

double dt;

double error, errSum, compP, compI, compD, lastError, pwm;

double dAngle = 0;

double PIDY, PIDZ;
float kp = 0.5;
float ki = 0;
float kd = 0.2;

double p_errorY, p_errorZ;
double errorZ, errorY;
double lastErrorZ, lastErrorY;

double d_angleY = 0;
double d_angleZ = 0;
 
float pwmY, pwmZ;

double pidY_p, pidY_i, pidY_d;
double pidZ_p, pidZ_i, pidZ_d;

int SGR = 3; //  Servo Gear Ratio
float servo_off;


double LocalOrientationZ, LocalOrientationY, LocalOrientationX;


int serialCounter = 0;

double compPZ, compIZ, compDZ;
double compPY, compIY, compDY;

double pErrorZ, pErrorY;
double errSumZ, errSumY;
double InputZ, InputY;
double lastInputZ, lastInputY;
double dErrorZ, dErrorY;
double dInputZ, dInputY;
double pIntGyroZ, pIntGyroY;
double dIntGyroZ, dIntGyroY;

uint64_t lastMicros;
uint64_t currentMicros;

// Perry Lib Functions //

Orientation ori;
EulerAngles gyroData;
EulerAngles gyroOut;

// =================== //

int servo_homeZ = 20;
int servo_homeY = 20;

int negCapVal;


// =========== GNC ========= // 

class PID
{
public:
    double Kp = 0, Ki = 0, Kd = 0;
    double integral = 0;
    double setpoint = 0;
    double input;

    PID() {  }; // Default initializer
    PID(double p, double i, double d) { Kp = p, Ki = i; Kd = d; }; // Gains initializer
    PID(double p, double i, double d, double s) { Kp = p, Ki = i; Kd = d; setpoint = s; }; // Full initializer

    double update(double input, double dt); // Updates PID maths and returns new output
    
private:
    double prevError = 0;
};

double PID::update(double input, double dt)
{
    double error = setpoint - input;
    integral += error * dt;

    double derivative = (error - prevError) / dt;
    prevError = error;
    return (Kp * error) + (Ki * integral) + (Kd * derivative);
}

PID zAxis {kp, ki, kd, 0};
PID yAxis {kp, ki, kd, 0};

// ========================= //



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

void capPwmVals(double valZ, double valY, int capVal) 
{
    negCapVal = -capVal;
    if (valZ > 0 && valZ > capVal)
    {
        valZ = capVal;
    } else if (valY > 0 && valY > capVal)
    {
        valY = capVal;
    } else if (valZ < negCapVal)
    {
        valZ = -30;
    } else if (valY < negCapVal)
    {
        valY = -30;
    }
}

// ============= //

double gpI;
double gyI;

// cumErrorX += errorX * elapsedTime;

#endif