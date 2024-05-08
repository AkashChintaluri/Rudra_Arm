#include <basicMPU6050.h> 
#include <ezButton.h>
#include <Servo.h>
#include <Wire.h>

basicMPU6050<> mpu;
Servo ct1;
ezButton limitSwitch(7);

// PID constants
const float Kp = 2.7;
const float Ki = 0.15;
const float Kd = 0.4;

// PID variables
float previousError = 0;
float integral = 0;
float derivative = 0;

// MPU Values
float gyroY = 0;
float accelY = 0;
float angleY = 0;
const float alpha = 0.9868;

// Initializing
unsigned long previousTime = 0;
float dt = 0;
const int cytronPin = 9;
int loop_check = 0;
int desiredAngle = 0;

void setup() {

  Serial.begin(9600);
  limitSwitch.setDebounceTime(50);

  ct1.attach(cytronPin);
  ct1.writeMicroseconds(1500);

}

void loop() {

  limitSwitch.loop();

  if(loop_check == 0){

    ct1.writeMicroseconds(1000);
    if(limitSwitch.getState() == LOW) {
      
      Serial.println("Pressed");
      ct1.writeMicroseconds(1500);
      Wire.begin();
      mpu.setup();

      loop_check++;
      return;
    }

    delay(100);
  }
  else {
    
    unsigned long currentTime = millis();
    dt = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;

    angleY += (mpu.gy() * (180 / PI)) * dt;
    int roundedAngleY = round(angleY);

    if (Serial.available() > 0 && desiredAngle == 0) {
        desiredAngle = Serial.parseInt();
    }

    if(desiredAngle != 0){
      
      float error = desiredAngle - roundedAngleY;
      integral += error * dt;
      derivative = (error - previousError) / dt;
      float output = Kp*error + Ki*integral + Kd*derivative;

      if (abs(error) < 1) {
          Serial.println("Desired angle achieved!");
          ct1.writeMicroseconds(1500);
      }
      else {
        
        int motorCommand;
        if (output >= 0)
            motorCommand = map(output, 0, 360, 1500, 1700);
        else
            motorCommand = map(abs(output), 0, 360, 1500, 1300);

        ct1.writeMicroseconds(motorCommand);
        
        Serial.print("Current Angle: ");
        Serial.println(roundedAngleY);
        Serial.print("Desired Angle: ");
        Serial.println(desiredAngle);

        previousError = error;

      }
    }
    else{
      
      Serial.print("Current Angle: ");
      Serial.println(roundedAngleY);
    }
  }
  delay(100);
}
