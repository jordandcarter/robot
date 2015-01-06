#include <Wire.h>                                     // Gyro SDA SCL library
#include "I2Cdev.h"
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h"
#include "CalLib.h"
#include <EEPROM.h>
#include "Pid.h"
#include <Servo.h>


RTIMU *imu;                                           // the IMU object
RTFusionRTQF fusion;                                  // the fusion object
RTIMUSettings settings;                               // the settings object

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port
#define  SERIAL_PORT_SPEED  115200

unsigned long lastRate;

unsigned long buzzTime = 0;
unsigned long silentTime = 0;
boolean buzz = true;
double currentPitch = 0;

Pid pitchPid;

Servo leftMotor;
Servo rightMotor;

double motorLimiter = 0.4;

RTVector3 q;

unsigned long now;
unsigned long previousNow;

void setup()
{
  int errcode;

  Serial.begin(SERIAL_PORT_SPEED);
  Wire.begin();
  imu = RTIMU::createIMU(&settings);                        // create the imu object

  if ((errcode = imu->IMUInit()) < 0) {
    //  Display error somehow
  }

  leftMotor.write(90);
  leftMotor.attach(6);
  rightMotor.write(90);
  rightMotor.attach(7);

  //if (imu->getCalibrationValid()){
  //  Serial.println("Using compass calibration");
  //} else {
  //  Serial.println("No valid compass calibration data");
  //}

  pitchPid = Pid(1, 0.01, 5);  //http://en.wikipedia.org/wiki/PID_controller#Manual_tuning

  now = previousNow = lastRate = millis();
}

double dt;

void loop()
{
  now = millis();
  dt = now - previousNow;
  tryReadIMU();
  pitchPid.loop(dt, 0, currentPitch);
  //Serial.println(pitchPid.output());
  if (upright(currentPitch)){
    setMotors(pitchPid.output());
  } else {
    stopMotors();
  }
  
  //Serial.println(dt);

  //buzzer();

  previousNow = now;
}

boolean upright(double pitch)
{
  return pitch < 50 && pitch > -50;
}

void setMotors(double s)
{
  int right = (int)motorSpeed(motorLimiter, s);
  int left = 90 - (right - 90);
  rightMotor.write(right+3);
  leftMotor.write(left+3);
  Serial.print(left);
  Serial.print(" | ");
  Serial.println(right);
}

void stopMotors()
{
  Serial.println("stopMotors");
  leftMotor.write(93);
  rightMotor.write(93);
}

double motorSpeed(double limiter, double speed)
{
  // Adjustable failsafe
  if (speed > 90*limiter){
    speed = 90*limiter;
  } else if (speed < -90*limiter){
    speed = -90*limiter;
  }

  // Hard failsafe
  if (speed > 90){
    speed = 90;
  } else if (speed < -90){
    speed = -90;
  }

  return 90 + speed;
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

    currentPitch = double(q.y() * 57.2957795) + 4;
    //Serial.println(currentPitch);
  }
}

void buzzer()
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
