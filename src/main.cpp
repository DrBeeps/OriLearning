#include "includes.h"

// Math Vars //
#define pi 3.14159265358979
// ========= //


// ========== GNC ========== //

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
// ======== // 

// ========================= // 



// ===== PID ===== //
float kp = 0.3;
float ki = 0.055;
float kd = 0.2;

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