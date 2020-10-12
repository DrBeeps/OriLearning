#include "includes.h"
#include "pid.h"

// Math Vars //
#define pi 3.14159265358979
// ========= //


// ========== GNC Vars ========== //

// Quats and Eulers //
Orientation ori;
EulerAngles gyroData;
EulerAngles gyroOut;
// ================ // 

// Time //
double dt;
uint64_t lastMicros;
uint64_t currentMicros;
int serialCounter = 0;
// ==== // 

// IMU Vars //
double IMUValX, IMUValY, IMUValZ; // raw offset values
double IMUOffsetX, IMUOffsetY, IMUOffsetZ; // final offset values
int IMUSampleTime;
// ======== //

// GNC Misc // 
double dAngle = 0;
double LocalOrientationZ, LocalOrientationY, LocalOrientationX;
float cs;
float sn;
float trueYOut, trueZOut;
int negCap;
// ======== // 

// ============================== // 

// ===== PID ===== //
float kp = 0.05;
float ki = 0.035;
float kd = 0.075;

PID zAxis = {kp, ki, kd, 0};
PID yAxis = {kp, ki, kd, 0};

double pidY_p, pidY_i, pidY_d;
double pidZ_p, pidZ_i, pidZ_d;
// =============== // 



// ===== Hardware ===== //

// IMU Hardware //
Bmi088Accel accel(Wire,0x19);
Bmi088Gyro gyro(Wire,0x69);
// ============ //

// Servo //
Servo servoY;
Servo servoZ;
int SGR = 3; //  Servo Gear Ratio
float servo_off;
float pwmY, pwmZ;
int servo_homeZ = 40;
int servo_homeY = 40;
// ===== // 

// ==================== //


// ========== GNC FUNCTIONS ========== //

int capVal(int val, int cap)
{
  negCap = (cap*-1);
  if(val > cap)
  {
    val = cap;
    return val;
  } else if (val < negCap)
  {
    val = negCap;
    return val;
  } else
  {
    return val;
  }
}

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
  
  trueZOut = pwmY * sn + pwmZ * cs;
  trueYOut = pwmY * cs - pwmZ * sn;

  trueZOut = (int)trueZOut * SGR;
  trueZOut = capVal(trueZOut, 30);
  trueYOut = (int)trueYOut * SGR;
  trueYOut = capVal(trueYOut, 30);

  Serial.print("PWM Z => "); Serial.print(trueYOut); Serial.print("\n");
  Serial.print("PWM Y => "); Serial.print(trueZOut); Serial.print("\n");

  servoZ.write(trueZOut);
  servoY.write(trueYOut);
}
// =================================== //



// ========== IMU Functions ========== //
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
// =================================== //



// ===== Servo Extras ===== //
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
// ======================== //


// ========== RUNNING FUNCTIONS ========== // 
void setup()
 {
  Serial.begin(9600);
  setupIMU();
  servoZ.attach(37);
  servoY.attach(36);
  lastMicros = micros();
  servoHome();
  zeroIMUDeg(dt);
  delay(2000);
} 
 
void loop()  
{
  currentMicros = micros();
  double dt = ((double)(currentMicros - lastMicros) / 1000000.);  
  Serial.print("dt =>"); Serial.print(dt); Serial.print("\n");
  stabilize(dt);
  lastMicros = currentMicros;
}
// ======================================= //