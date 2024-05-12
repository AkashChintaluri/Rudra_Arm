#include <basicMPU6050.h> 
#include <ezButton.h>
#include <Servo.h>
#include <Wire.h>

basicMPU6050<> mpu;
Servo ct;
ezButton limitSwitch1(7);
ezButton limitSwitch2(6);

// PID constants
const float Kp = 2.7;
const float Ki = 0.15;
const float Kd = 0.4;

// PID variables
float previousError = 0;
float integral = 0;
float derivative = 0;

// MPU Values
float angleY_gyro = 0;
float angleY_accel = 0;
float angleY = 0;
float gyroOffset = 0;
const float alpha = 0.02;

// Initializing
unsigned long previousTime = 0;
float dt = 0;
const int cytronPin = 9;
int loop_check = 0;
int desiredAngle = 0;

void setup() {

  Serial.begin(9600);
  limitSwitch.setDebounceTime(50);
  mpu.setup();

  ct.attach(cytronPin);
  ct.writeMicroseconds(1500);

  for (int i = 0; i < 1000; i++) {
    gyroOffset += mpu.gy();
    delay(1);
  }
  gyroOffset /= 1000;

}

void loop() {
  unsigned long currentTime = millis();
  dt = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;
  
  float accelX = mpu.ax() * 9.81;
  float accelZ = mpu.az() * 9.81;

  angleY_accel = ((atan2(-accelX, -accelZ) + PI) * (180 / PI));
  if(angleY_accel >= 180 ) {
    angleY_accel -= 360;
  }

  angleY_gyro += ((mpu.gy() - gyroOffset) * (180 / PI)) * dt;

  angleY = alpha * (angleY + angleY_gyro) + (1 - alpha) * angleY_accel;
  int roundedAngleY = round(angleY);
  Serial.print("Rounded Angle: ");
  Serial.println(roundedAngleY);

  if (Serial.available() > 0) {
    desiredAngle = Serial.parseInt();
  }

  float error = desiredAngle - roundedAngleY;

  if(abs(error <= 90)){
    if(abs(error) > 1){

      integral += error * dt;
      derivative = (error - previousError) / dt;
      float output = Kp * error + Ki * integral + Kd * derivative;

      int motorCommand;

      if (output >= 0)
          motorCommand = map(output, 0, 360, 1500, 1700);
      else
          motorCommand = map(abs(output), 0, 360, 1500, 1300);
      
      ct.writeMicroseconds(motorCommand);
        
      previousError = error;

      if(limitSwitch1.isPressed()){
        Serial.println("Cannot move, limit switch is pressed");
        ct.writeMicroseconds(1600);
        delay(10);
        desiredAngle = roundedAngleY;
      }
      else if(limitSwitch2.isPressed()){
        Serial.println("Cannot move, limit switch is pressed");
        ct.writeMicroseconds(1400);
        delay(10);
        desiredAngle = roundedAngleY;
      }
    }
    else{
      Serial.println("Desired angle achieved!");
      ct.writeMicroseconds(1500);
    }
  }
  else{
    Serial.println("Incorrect IMU position, please adjust the IMU position.");
    ct.writeMicroseconds(1500);
  }

  ct.writeMicroseconds(1500);
}