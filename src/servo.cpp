#include "includes.h"

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