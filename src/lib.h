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

double dAngle = 0;

float kp = 0.3;
float ki = 0.055;
float kd = 0.2;

double d_angleY = 0;
double d_angleZ = 0;
 
float pwmY, pwmZ;

double pidY_p, pidY_i, pidY_d;
double pidZ_p, pidZ_i, pidZ_d;

int SGR = 3; //  Servo Gear Ratio
float servo_off;


double LocalOrientationZ, LocalOrientationY, LocalOrientationX;


int serialCounter = 0;

uint64_t lastMicros;
uint64_t currentMicros;

// Perry Lib Functions //

Orientation ori;
EulerAngles gyroData;
EulerAngles gyroOut;

// =================== //

int servo_homeZ = 40;
int servo_homeY = 40;

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

float cs;
float sn;
float trueYOut, trueZOut;

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

double IMUValX, IMUValY, IMUValZ; // raw offset values
double IMUOffsetX, IMUOffsetY, IMUOffsetZ; // final offset values
int IMUSampleTime;

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
#endif