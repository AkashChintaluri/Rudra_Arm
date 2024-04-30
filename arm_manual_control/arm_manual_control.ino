#include <Wire.h>
#include <Servo.h>
#include <MPU6050.h>

const int sabertoothPin = 9;

float gyroY = 0;
float angleY = 0;

int input = 1500;

unsigned long previousTime = 0;
float deltaTime = 0;

MPU6050 mpu;
Servo sb1;

void setup() {
  Serial.begin(9600);
  
  Wire.begin();
  //mpu.initialize();
  //mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  //pinMode(sabertoothPin, OUTPUT);
  sb1.attach(sabertoothPin);
}

void loop() {

  /*unsigned long currentTime = millis();
  deltaTime = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;
  int16_t gyroRaw[3];
  mpu.getRotation(&gyroRaw[0], &gyroRaw[1], &gyroRaw[2]);
  
  gyroY = gyroRaw[1] / 65.5;
    
  angleY += (gyroY * deltaTime);
  int roundedAngleY = round(angleY);
  
  Serial.print("Current Angle: ");
  Serial.println(roundedAngleY);*/

  if (Serial.available() > 0){
    input = Serial.parseInt();
  }

  Serial.println(input);
  sb1.writeMicroseconds(input);
  //analogWrite(sabertoothPin, input);
}
