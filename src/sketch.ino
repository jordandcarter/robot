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
Pid positionPid;

Sabertooth ST(128);

double motorLimiter = 0.7;

RTVector3 q;

unsigned long now;
unsigned long previousNow;

double centrePoint = 5.5;
double rightBias = 4;
double leftBias = 3;
boolean useSerial = false;

#define leftEncoder1 2 // White wire
#define leftEncoder2 4 // Yellow wire
#define rightEncoder1 3 // Yellow wire
#define rightEncoder2 5 // White wire

volatile long leftCounter = 0;
volatile long rightCounter = 0;

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

  //if (imu->getCalibrationValid()){
  //  Serial.println("Using compass calibration");
  //} else {
  //  Serial.println("No valid compass calibration data");
  //}

  //http://en.wikipedia.org/wiki/PID_controller#Manual_tuning
  pitchPid = Pid(11, 0, 100);
  positionPid = Pid(0,0,0);//0.0005, 0, 0);
  
  lastRate = millis();
  previousNow = now = micros();
  digitalWrite(8, HIGH);
  delay(50);
  digitalWrite(8, LOW);
  delay(1000);
  digitalWrite(8, HIGH);
  delay(100);
  digitalWrite(8, LOW);
  
  pinMode(leftEncoder1,INPUT);
  pinMode(leftEncoder2,INPUT);
  pinMode(rightEncoder1,INPUT);
  pinMode(rightEncoder2,INPUT); 
  attachInterrupt(0,leftEncoder,FALLING); // pin 2
  attachInterrupt(1,rightEncoder,FALLING); // pin 3
  SabertoothTXPinSerial.begin(38400); // Serial1
  ST.setRamping(1);
  ST.drive(0); // The Sabertooth won't act on mixed mode packet serial commands until
  ST.turn(0);
}

double dt;
int initing = 0;
boolean initd = false;

double positionOutput = 0;
int   STD_LOOP_TIME  =  9;
int lastLoopTime = STD_LOOP_TIME;
int lastLoopUsefulTime = STD_LOOP_TIME;
unsigned long loopStartTime = 0;

void loop()
{
  if(initd){
    now = micros();
    dt = now - previousNow;
    tryReadIMU();
    positionPid.loop(dt, 0, (leftCounter + rightCounter)/2);
    positionOutput = positionPid.output();
    if (positionOutput > 15){
      positionOutput = 15;
    } else if (positionOutput < -15){
      positionOutput = -15;
    }
    pitchPid.loop(dt, -positionOutput, -currentPitch);
    if (useSerial){
      Serial.println(pitchPid.output());
      //Serial.print(leftCounter);
      //Serial.print(" | ");
      //Serial.println(rightCounter);
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
    if (millis() > 3000){
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
  lastLoopUsefulTime = millis()-loopStartTime;
  if(lastLoopUsefulTime<STD_LOOP_TIME)         delay(STD_LOOP_TIME-lastLoopUsefulTime);
  lastLoopTime = millis() - loopStartTime;
  loopStartTime = millis();
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
    //Serial.println(speed);
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

/* Interrupt routine and encoder read functions - I read using the port registers for faster processing */
void leftEncoder() {
  if(PING & 0b00100000){ // read pin 4
    leftCounter++;
  } else {
    leftCounter--;
  }    
}
void rightEncoder() {
  if(PINE & 0b00001000){ // read pin 5
    rightCounter++;
  } else {
    rightCounter--;
  }  
}
