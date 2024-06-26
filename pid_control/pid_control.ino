#include <Wire.h>
#include <Servo.h>
#include <basicMPU6050.h>
#include <ezButton.h>

basicMPU6050<> mpu;
Servo ct1;
ezButton limitSwitch(7);

unsigned long previousTime = 0;
float deltaTime = 0;

float gyroY = 0;
float accelY = 0;
float angleY = 0;
float alpha = 0.9868; //Complemenatary Filter

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
const int cytronPin = 9;

float offset = 0;
float g_offset = 11;

int loop_check = 0;

void setup() {
  
    Serial.begin(9600);
    limitSwitch.setDebounceTime(50);

    Wire.begin();
    mpu.setup();
  
    ct1.attach(cytronPin);
    ct1.writeMicroseconds(1500);
}

void loop() {

    limitSwitch.loop();

    if(loop_check == 0){
      ct1.writeMicroseconds(1000);
      Serial.println(1000);
      if(limitSwitch.getState() == LOW){

        Serial.println("Pressed");

        unsigned long currentTime = millis();
        deltaTime = (currentTime - previousTime) / 1000.0;
        previousTime = currentTime;
        
        //offset = alpha * (offset + (mpu.gy() * (180 / PI)) * deltaTime) + (0.0132)*(mpu.ay() * 9.81);
        offset += (mpu.gy() * (180 / PI)) * deltaTime;

        ct1.writeMicroseconds(1500);

        loop_check = 1;

        previousTime = 0;
        deltaTime = 0;
        
        return;
      }
    }
    else{
      
      unsigned long currentTime = millis();
      deltaTime = (currentTime - previousTime) / 1000.0;
      previousTime = currentTime;
      
      //angleY = (alpha * (angleY + (mpu.gy() * (180 / PI)) * deltaTime) + (0.0132)*(mpu.ay() * 9.81)) - offset;
      angleY += (mpu.gy() * (180 / PI)) * deltaTime;
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
            ct1.writeMicroseconds(1500);
        }
    
        int motorCommand;
        if (output >= 0) {
            motorCommand = map(output, 0, 360, 1700, 2000);
        } else {
            motorCommand = map(abs(output), 0, 360, 1300, 1000);
        }
        
        //Serial.print("Motor Command: ");
        //Serial.println(motorCommand);
        
        ct1.writeMicroseconds(motorCommand);
    
        Serial.print("Current Angle: ");
        Serial.println(roundedAngleY);
        Serial.print("Desired Angle: ");
        Serial.println(desiredAngle);
        //Serial.print("PID output: ");
        //Serial.println(output);
        
        previousError = error;
        
      }
      else{
        Serial.print("Current Angle: ");
        Serial.println(roundedAngleY);
        //Serial.print("Offset: ");
        //Serial.println(offset);
        ct1.writeMicroseconds(1500);
      }
    }
  delay(100);
}
