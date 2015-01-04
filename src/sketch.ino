#include <Wire.h>
#include "I2Cdev.h"
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h" 
#include "CalLib.h"
#include <EEPROM.h>

RTIMU *imu;                                           // the IMU object
RTFusionRTQF fusion;                                  // the fusion object
RTIMUSettings settings;                               // the settings object

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port

#define  SERIAL_PORT_SPEED  115200

unsigned long lastDisplay;
unsigned long lastRate;
int sampleCount;

unsigned long timer;
unsigned long buzzTime;
unsigned long silentTime;
boolean buzz = true;

void setup()
{
  int errcode;
  
  Serial.begin(SERIAL_PORT_SPEED);
  Wire.begin();
  imu = RTIMU::createIMU(&settings);                        // create the imu object
  
  //Serial.print("ArduinoIMU starting using device "); Serial.println(imu->IMUName());
  if ((errcode = imu->IMUInit()) < 0) {
    //Serial.print("Failed to init IMU: "); Serial.println(errcode);
  }
  
  //if (imu->getCalibrationValid())
    //Serial.println("Using compass calibration");
  //else
    //Serial.println("No valid compass calibration data");

  lastDisplay = lastRate = millis();
  sampleCount = 0;
  timer = millis();
}

RTVector3 q;

void loop()
{  
  unsigned long now = millis();
  unsigned long delta;
  
  if (imu->IMURead()) {                                // get the latest data if ready yet
    fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
    sampleCount++;
    if ((delta = now - lastRate) >= 1000) {
      //Serial.print("Sample rate: "); Serial.print(sampleCount);
      imu->IMUGyroBiasValid();
        //Serial.println(", gyro bias valid");
      //else
        //Serial.println(", calculating gyro bias - don't move IMU!!");
        
      sampleCount = 0;
      lastRate = now;
    }
    //if ((now - lastDisplay) >= DISPLAY_INTERVAL) {
      //lastDisplay = now;
//      RTMath::display("Gyro:", (RTVector3&)imu->getGyro());                // gyro data
//      RTMath::display("Accel:", (RTVector3&)imu->getAccel());              // accel data
//      RTMath::display("Mag:", (RTVector3&)imu->getCompass());              // compass data
//      RTMath::displayDegrees("", (RTVector3&)fusion.getFusionPose()); // fused output
//      Serial.println();
      q = (RTVector3&)fusion.getFusionPose();
      
      Serial.print(q.y() * 57.2957795);
      Serial.println();
    //}
  }
  
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
