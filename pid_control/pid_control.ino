#include <Wire.h>
#include <Servo.h>
#include <MPU6050.h>
#include <ezButton.h>

MPU6050 mpu;
Servo sb1;
ezButton limitSwitch(7);

unsigned long previousTime = 0;
float deltaTime = 0;

float gyroY = 0;
float angleY = 0;

// PID constants
float Kp = 2.7;
float Ki = 0.15;
float Kd = 0.4;

// PID variables
float previousError = 0;
float integral = 0;
float derivative = 0;

// Desired angle
int desiredAngle = 0;

// Sabertooth pins
const int sabertoothPin1 = 9;

float offset = 0;
float g_offset = 11;

int loop_check = 0;

void setup() {
  
    Serial.begin(9600);
    limitSwitch.setDebounceTime(50);

    Wire.begin();
    mpu.initialize();
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  
    sb1.attach(sabertoothPin1);
    sb1.writeMicroseconds(1500);
}

void loop() {

    limitSwitch.loop();

    if(loop_check == 0){
      sb1.writeMicroseconds(1000);
      Serial.println(1000);
      if(limitSwitch.getState() == LOW){

        Serial.print("Pressed");
        
        int16_t gyroRaw[3];
        mpu.getRotation(&gyroRaw[0], &gyroRaw[1], &gyroRaw[2]);
      
        float offsetY = gyroRaw[1] / 65.5;
        offset += (gyroY * deltaTime);

        sb1.writeMicroseconds(1500);

        loop_check = 1;
        return;
      }
    }
    else{
      
      unsigned long currentTime = millis();
      deltaTime = (currentTime - previousTime) / 1000.0;
      previousTime = currentTime;
    
      int16_t gyroRaw[3];
      mpu.getRotation(&gyroRaw[0], &gyroRaw[1], &gyroRaw[2]);
    
      gyroY = gyroRaw[1] / 65.5;
      
      angleY += (gyroY * deltaTime) - offset;
      int roundedAngleY = round(angleY);
      
    
      if (Serial.available() > 0 && desiredAngle == 0) {
        desiredAngle = Serial.parseInt();
        desiredAngle = desiredAngle;
      }
  
      if(desiredAngle!=0){
  
        float error = desiredAngle - roundedAngleY;
        integral += error * deltaTime;
        derivative = (error - previousError) / deltaTime;
        float output = Kp*error + Ki*integral + Kd*derivative;
  
        if (abs(error) < 1) {
            Serial.println("Desired angle achieved!");
            sb1.writeMicroseconds(1500);
        }
    
        int motorCommand;
        if (output >= 0) {
            motorCommand = map(output, 0, 360, 1700, 2000);
        } else {
            motorCommand = map(abs(output), 0, 360, 1300, 1000);
        }
        
        Serial.print("Motor Command: ");
        Serial.println(motorCommand);
        
        sb1.writeMicroseconds(motorCommand);
    
        Serial.print("Current Angle: ");
        Serial.println(roundedAngleY);
        Serial.print("Desired Angle: ");
        Serial.println(desiredAngle);
        Serial.print("PID output: ");
        Serial.println(output);
        
        previousError = error;
        
      }
      else{
        Serial.print("Current Angle: ");
        Serial.println(roundedAngleY);
        sb1.writeMicroseconds(1500);
      }
    }
  delay(100);
}
