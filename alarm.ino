#include <Servo.h>
#include <Wire.h>
#include "Kalman.h"

Servo servo;
Kalman kalmanX;
Kalman kalmanY;
Kalman kalmanZ;

uint8_t IMUAddress = 0x68;

int16_t accX;
int16_t accY;
int16_t accZ;
int16_t tempRaw;
int16_t gyroX;
int16_t gyroY;
int16_t gyroZ;
double accXangle;
double accYangle;
double accZangle;
double temp;
double gyroXangle = 180;
double gyroYangle = 180;
double gyroZangle = 180;
double compAngleX = 180;
double compAngleY = 180;
double compAngleZ = 180;
double kalAngleX;
double kalAngleY;
double kalAngleZ;
uint32_t timer;

int resultX;
int resultY;
int resultT;
int savedX = 0;
int savedY = 0;

byte ledPin = 13;
byte sensorPin0 = 2;
byte sensorPin1 = 3;
byte servoPin = 5;
byte buttonPin = 4;

byte alarmIsOn = 0;


void setup () {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  pinMode(servoPin, OUTPUT);
  pinMode(sensorPin0, INPUT);
  pinMode(sensorPin1, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(0, alarm, RISING);
  attachInterrupt(1, alarm, RISING);
  Wire.begin();
  i2cWrite(0x6B,0x00);
  kalmanX.setAngle(180);
  kalmanY.setAngle(180);
  kalmanZ.setAngle(180);
  timer = micros();
}
void loop () {
  checkButton();
  if ( alarmIsOn ) {
    checkAngles();
  }
}

void checkButton () {
  if ( !digitalRead(buttonPin) ) {
    if ( alarmIsOn ) {
      alarmOff();
    } else {
      alarmOn();
    }
  }
}

void alarmOff () {
  digitalWrite(ledPin, LOW);
  servoOpen();
  alarmIsOn = 0;
}

void alarmOn () {
  saveAngles();
  servoClose();
  alarmIsOn = 1;
}

void checkAngles () {
  gyRead();
  if (  resultX > savedX + 10 || resultX < savedX - 10 || resultY > savedY + 10 || resultY < savedY - 10 ) {
    alarm();
  }
}

void saveAngles () {
  gyRead();
  savedX = resultX;
  savedY = resultY;
}

void alarm () {
  if ( alarmIsOn ) {
    digitalWrite(ledPin, HIGH);
  }
}

void servoClose () {
  servo.attach(servoPin);
  servo.write(0);
  delay(500);
  servo.detach();
}

void servoOpen () {
  servo.attach(servoPin);
  servo.write(180);
  delay(500);
  servo.detach();
}


void i2cWrite (uint8_t registerAddress, uint8_t data){
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t* i2cRead(uint8_t registerAddress, uint8_t nbytes) {
  uint8_t data[nbytes];
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.endTransmission(false);
  Wire.requestFrom(IMUAddress, nbytes);
  for(uint8_t i = 0; i < nbytes; i++)
    data [i]= Wire.read();
  return data;
}

void gyRead () {
  int i = 5;
  while ( --i > 0 ) {
    uint8_t* data = i2cRead(0x3B,14);
    accX = ((data[0] << 8) | data[1]);
    accY = ((data[2] << 8) | data[3]);
    accZ = ((data[4] << 8) | data[5]);
    tempRaw = ((data[6] << 8) | data[7]);
    gyroX = ((data[8] << 8) | data[9]);
    gyroY = ((data[10] << 8) | data[11]);
    gyroZ = ((data[12] << 8) | data[13]);

    accYangle = (atan2(accX,accZ)+PI)*RAD_TO_DEG;
    accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;  
    accZangle = (atan2(accY,accX)+PI)*RAD_TO_DEG;
    double gyroXrate = (double)gyroX/131.0;
    double gyroYrate = -((double)gyroY/131.0);
    gyroXangle += kalmanX.getRate()*((double)(micros()-timer)/1000000);
    gyroYangle += kalmanY.getRate()*((double)(micros()-timer)/1000000);
    kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros()-timer)/1000000);
    kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros()-timer)/1000000);
    timer = micros();
    delay(30);
  }
  resultX = kalAngleX;
  resultY = kalAngleY;
  resultT = tempRaw/340.00+36.53;
}
