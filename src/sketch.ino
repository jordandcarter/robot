#include <Sabertooth.h>

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

unsigned long lastRate;

unsigned long buzzTime = 0;
unsigned long silentTime = 0;
boolean buzz = true;
double currentPitch = 0;

Pid pitchPid;

Sabertooth ST(128);

double motorLimiter = 0.6;

RTVector3 q;

unsigned long now;
unsigned long previousNow;

double centrePoint = 5.5;
double rightBias = 4;
double leftBias = 3;
boolean useSerial = false;

void setup()
{
  int errcode;
   
  if (useSerial){
    Serial.begin(SERIAL_PORT_SPEED);
  }
  Wire.begin();
  imu = RTIMU::createIMU(&settings);                        // create the imu object

  if ((errcode = imu->IMUInit()) < 0) {
    //  Display error somehow
  }

  SabertoothTXPinSerial.begin(38400); // Serial1
  delay(50);
  ST.setRamping(1);
  ST.drive(0); // The Sabertooth won't act on mixed mode packet serial commands until
  ST.turn(0);

  //if (imu->getCalibrationValid()){
  //  Serial.println("Using compass calibration");
  //} else {
  //  Serial.println("No valid compass calibration data");
  //}

  //http://en.wikipedia.org/wiki/PID_controller#Manual_tuning
  pitchPid = Pid(5, 0.00005, 0);
  
  lastRate = millis();
  previousNow = now = micros();
  digitalWrite(8, HIGH);
  delay(50);
  digitalWrite(8, LOW);
  delay(1000);
  digitalWrite(8, HIGH);
  delay(100);
  digitalWrite(8, LOW);
}

double dt;
int initing = 0;
boolean initd = false;

void loop()
{
  if(initd){
    now = micros();
    dt = now - previousNow;
    tryReadIMU();
    pitchPid.loop(dt, 0, currentPitch);
    if (useSerial){
      Serial.println(currentPitch);
    }
    if (upright(currentPitch)){
      setMotors(pitchPid.output());
    } else {
      stopMotors();
    }
    
    //Serial.println(dt);
  
    //buzzer();
  
    previousNow = now;
  } else {
    tryReadIMU();
    initing++;
    if (initing > 1000){
      initd = true;
      digitalWrite(8, HIGH);
      delay(60);
      digitalWrite(8, LOW);
      delay(10);
      digitalWrite(8, HIGH);
      delay(25);
      digitalWrite(8, LOW);
      delay(10);
      digitalWrite(8, HIGH);
      delay(60);
      digitalWrite(8, LOW);
      delay(10);
      digitalWrite(8, HIGH);
      delay(25);
      digitalWrite(8, LOW);
      delay(10);
      digitalWrite(8, HIGH);
      delay(60);
      digitalWrite(8, LOW);
      delay(10);
      digitalWrite(8, HIGH);
      delay(25);
      digitalWrite(8, LOW);
      delay(10);  
    }
  }
 }

boolean upright(double pitch)
{
  return pitch < 20 && pitch > -20;
}

void setMotors(double s)
{
  int speed = (int)motorSpeed(motorLimiter, s);
  ST.drive(speed);
  //ST.motor(1, 5);//right reverse
  //ST.motor(2, -5);//left reverse
  //ST.motor(1, -5);//right fwd
  //ST.motor(2, 5);//left fwd
  if (useSerial){
    Serial.print(speed);
  }
}

void stopMotors()
{
  if (useSerial){
    Serial.println("stopMotors");
  }
  ST.drive(0);
}

double motorSpeed(double limiter, double speed)
{
  // Adjustable failsafe
  if (speed > 127*limiter){
    speed = 127*limiter;
  } else if (speed < -127*limiter){
    speed = -127*limiter;
  }

  // Hard failsafe
  if (speed > 127){
    speed = 127;
  } else if (speed < -127){
    speed = -127;
  }
  return speed;
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

    currentPitch = double(q.y() * 57.2957795) + centrePoint;
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

