/******************************************************************************
MinimalistExample.ino

Marshall Taylor @ SparkFun Electronics
May 20, 2015
https://github.com/sparkfun/LSM6DS3_Breakout
https://github.com/sparkfun/SparkFun_LSM6DS3_Arduino_Library

Description:
Most basic example of use.

Example using the LSM6DS3 with basic settings.  This sketch collects Gyro and
Accelerometer data every second, then presents it on the serial monitor.

Resources:
Uses Wire.h for i2c operation
Uses SPI.h for SPI operation
Either can be omitted if not used

Development environment specifics:
Arduino IDE 1.6.4
Teensy loader 1.23

Hardware connections:
Connect I2C SDA line to A4
Connect I2C SCL line to A5
Connect GND and 3.3v power to the IMU

This code is released under the [MIT License](http://opensource.org/licenses/MIT).

Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.

Distributed as-is; no warranty is given.
******************************************************************************/

#include "SparkFunLSM6DS3.h"
#include "Wire.h"
#include "SPI.h"

LSM6DS3 myIMU; //Default constructor is I2C, addr 0x6B
const int pwmspeed = 3 ;     //initializing pin 3 as ‘pwm’ variable
const int pwmturn = 5 ;     //initializing pin 5 as ‘pwm’ variable

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(1000); //relax...
  Serial.println("Processor came out of reset.\n");
  
  //Call .begin() to configure the IMU
  myIMU.begin();

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(pwmspeed,OUTPUT) ;   //Set pin 3 as output
  pinMode(pwmturn,OUTPUT) ;   //Set pin 3 as output  
  
  
}


void loop()
{
  //Get all parameters
  float xaccel = myIMU.readFloatAccelX();
  float yaccel = myIMU.readFloatAccelY();
  float zaccel = myIMU.readFloatAccelZ();
  int xindigital = 0;
  int yindigital = 0;
  if (xaccel > 0 && zaccel > 0)
  {
    xindigital = 0;
  }
  else
  {
    xindigital = (int)(((xaccel - 1.0)/(0.0 - 1.0))*(255-0) + 0);
    if (xindigital < 0)
    {
      xindigital = 0;
    }
  }

  if (zaccel < 0)
  {
    yindigital = 19;
  }
  else
  {
    yindigital = (int)(((yaccel - 1.0)/(-1 - 1.0))*(239-121) + 121);
  }

  
   
  Serial.print("\nAccelerometer:\n");
  Serial.print(" X = ");
  Serial.println(xaccel, 4);
  Serial.print(" Xindigital = ");
  Serial.println(xindigital);
  Serial.print(" Yindigital = ");
  Serial.println(yindigital);
  Serial.print(" Y = ");
  Serial.println(yaccel, 4);
  Serial.print(" Z = ");
  Serial.println(zaccel, 4);

  analogWrite(pwmspeed, xindigital) ; 
  analogWrite(pwmturn, yindigital) ;
  //analogWrite(pwmturn, 239 121) ; 
  
  /*digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  if (on)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    on = 0;
  }
  else
  {
    digitalWrite(LED_BUILTIN, LOW);
    on = 1;
  }
  //analogWrite(outputPin, val)

  Serial.print("\nGyroscope:\n");
  Serial.print(" X = ");
  Serial.println(myIMU.readFloatGyroX(), 4);
  Serial.print(" Y = ");
  Serial.println(myIMU.readFloatGyroY(), 4);
  Serial.print(" Z = ");
  Serial.println(myIMU.readFloatGyroZ(), 4);

  Serial.print("\nThermometer:\n");
  Serial.print(" Degrees C = ");
  Serial.println(myIMU.readTempC(), 4);
  Serial.print(" Degrees F = ");
  Serial.println(myIMU.readTempF(), 4);*/
  
  delay(1000);
}
