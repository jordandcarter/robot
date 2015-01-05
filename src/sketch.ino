#include <Wire.h>                                     // Gyro SDA SCL library
#include "I2Cdev.h"
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h" 
#include "CalLib.h"
#include <EEPROM.h>
#include "Pid.h"

RTIMU *imu;                                           // the IMU object
RTFusionRTQF fusion;                                  // the fusion object
RTIMUSettings settings;                               // the settings object

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port
#define  SERIAL_PORT_SPEED  115200

unsigned long lastDisplay;
unsigned long lastRate;

unsigned long buzzTime = 0;
unsigned long silentTime = 0;
boolean buzz = true;

Pid pitchPid;

void setup()
{
  int errcode;
  
  Serial.begin(SERIAL_PORT_SPEED);
  Wire.begin();
  imu = RTIMU::createIMU(&settings);                        // create the imu object
  
  if ((errcode = imu->IMUInit()) < 0) {
    //  Display error somehow
  }
  
  //if (imu->getCalibrationValid())
    //Serial.println("Using compass calibration");
  //else
    //Serial.println("No valid compass calibration data");

  pitchPid = Pid::Pid(0, 0, 0)  //http://en.wikipedia.org/wiki/PID_controller#Manual_tuning

  lastDisplay = lastRate = millis();
}

RTVector3 q;

unsigned long now;

void loop()
{  
  now = millis();
  tryReadIMU();
  buzz();
}

void pid(double setpoint, )
{
  
}

void tryReadIMU()
{
  if (imu->IMURead()) {                                // get the latest data if ready yet
    fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
    if ((now - lastRate) >= 1000) {
      imu->IMUGyroBiasValid();
        //Serial.println(", gyro bias valid");
      //else
        //Serial.println(", calculating gyro bias - don't move IMU!!");
        
      lastRate = now;
    }

    q = (RTVector3&)fusion.getFusionPose();
    
    Serial.print(q.y() * 57.2957795);
    Serial.println();
  }  
}

void buzz()
{
  if (buzz && (now - buzzTime) >= abs((q.y() * 57.2957795))* 5){
    buzz = false;
    silentTime = now;
    digitalWrite(8, LOW);
  }
  if (!buzz && (now - silentTime) >= (1000-abs((q.y() * 57.2957795))* 10)){
    buzz = true;
    buzzTime = now;
    digitalWrite(8, HIGH);
  }
}
