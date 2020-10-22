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

// === PID Module Test === //

float inputAngleZ;
float inputAngleY;
float moduleTestOutputZ;
float moduleTestOutputY;

// ======================= //



// ===== Hardware ===== //

// IMU Hardware //
Bmi088Accel accel(Wire,0x19);
Bmi088Gyro gyro(Wire,0x69);
// ============ //

// Servo //
Servo servoY;
Servo servoZ;
int SGR = 3; //  Servo Gear Ratio
float servo_offset = 30;
float pwmY, pwmZ;
int servo_homeZ = 90;
int servo_homeY = 90;
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

void PIDModuleTest()
{
  Serial.println("---------- Welcome to the PID Module test ----------");
  Serial.println("Input the Z Angle...");
  inputAngleZ = Serial.parseFloat();
  delay(2000);
  Serial.println("Input the Y Angle...");
  inputAngleY = Serial.parseFloat();

  moduleTestOutputZ = zAxis.update(inputAngleZ, dt);
  moduleTestOutputY = yAxis.update(inputAngleY, dt);

  Serial.print("Output Z: "); Serial.print(moduleTestOutputZ); Serial.print("\t");
  Serial.print("Output Y: "); Serial.print(moduleTestOutputY); Serial.print("\t");
}

void stabilize(double dt) 
{
  gyro.readSensor();

  gyroData.roll = (gyro.getGyroX_rads());
  gyroData.pitch = (-gyro.getGyroZ_rads());
  gyroData.yaw = (-gyro.getGyroY_rads());

  ori.update(gyroData, dt);
  gyroOut = ori.toEuler();
  
  LocalOrientationX = (gyroOut.roll * RAD_TO_DEG);
  LocalOrientationY = (gyroOut.pitch * RAD_TO_DEG);
  LocalOrientationZ = (gyroOut.yaw * RAD_TO_DEG);

  LocalOrientationX -= IMUOffsetX;
  LocalOrientationY -= IMUOffsetY;
  LocalOrientationZ -= IMUOffsetZ;

  Serial.print("ORE Z => "); Serial.print(LocalOrientationZ); Serial.print("\n");
  Serial.print("ORE Y => "); Serial.print(LocalOrientationY); Serial.print("\n");

  pwmZ = zAxis.update(LocalOrientationZ, dt);
  pwmY = yAxis.update(LocalOrientationY, dt);

  delay(100);

  cs = cos(-gyroOut.roll);
  sn = sin(-gyroOut.roll);
  
  trueZOut = pwmY * sn + pwmZ * cs;
  trueYOut = pwmY * cs - pwmZ * sn;

  trueZOut = (constrain((int)(trueZOut * RAD_TO_DEG * SGR), -30, 30));
  trueYOut = (constrain((int)(trueYOut * RAD_TO_DEG * SGR), -30, 30));

  Serial.print("PWM Z => "); Serial.print(trueYOut); Serial.print("\n");
  Serial.print("PWM Y => "); Serial.print(trueZOut); Serial.print("\n");

  servoZ.write(90 + trueZOut);
  servoY.write(90 + trueYOut);
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

void zeroIMUDeg()
{
  Serial.println("Zeroing IMU...");
    for(IMUSampleTime = 0; IMUSampleTime < 10; IMUSampleTime++)
    {
        currentMicros = micros();
        dt = ((double)(currentMicros - lastMicros) / 1000000.); 

        getOrientation(dt);

        IMUValX += LocalOrientationX;
        IMUValY += LocalOrientationY;
        IMUValZ += LocalOrientationZ;

        lastMicros = currentMicros;
    }
    IMUOffsetX = IMUValX / 10;
    IMUOffsetY = IMUValY / 10;
    IMUOffsetZ = IMUValZ / 10;

    Serial.println("IMU zeroing finished... calculating offsets...");
    delay(1000);

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

void roughOffsetCalc(double LocalOrientationZ, double LocalOrientationY)
{
  IMUOffsetZ = 0 - LocalOrientationZ;
  IMUOffsetY = 0 - LocalOrientationY;
}

// ======================== //


// ========== RUNNING FUNCTIONS ========== // 
void setup()
 {
  Serial.begin(9600);
  setupIMU();
  servoZ.attach(37);
  servoY.attach(36);
  servoHome();
  delay(1000);
  zeroIMUDeg();
  delay(2000);
} 

void loop()  
{
  currentMicros = micros();
  dt = ((double)(currentMicros - lastMicros) / 1000000.);  
  Serial.print("dt =>"); Serial.print(dt); Serial.print("\n");
  stabilize(dt);
  lastMicros = currentMicros;
}
// ======================================= //


