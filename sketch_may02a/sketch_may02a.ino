#include <Wire.h>
#include <basicMPU6050.h>

basicMPU6050<> mpu;

float angleY_gyro = 0;
float angleY_accel = 0;
float angleY = 0;
unsigned long previousTime = 0;
float deltaTime = 0;
float alpha = 0.02;
float gyroOffset = 0;

void setup() {
  
    Serial.begin(9600);
    Wire.begin();
    mpu.setup();
    float testValue = mpu.gy();
    if (isnan(testValue) || abs(testValue) > 1000) {
        Serial.println("MPU6050 initialization failed");
        while (1); // halt execution
    }

    for (int i = 0; i < 1000; i++) {
        gyroOffset += mpu.gy();
        delay(1);
    }
    gyroOffset /= 1000;
}

void loop() {
  
    unsigned long currentTime = millis();
    deltaTime = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;
    
    float accelX = mpu.ax() * 9.81;
    float accelY = mpu.ay() * 9.81;
    float accelZ = mpu.az() * 9.81;
    
    angleY_accel = ((atan2(-accelX, -accelZ) + PI) * (180 / PI));
    if(angleY_accel >= 180 ) {
      angleY_accel -= 360;
    }
//    angleY_accel = (180/PI)*angleY_accel;
    angleY_gyro += ((mpu.gy() - gyroOffset) * (180 / PI)) * deltaTime;

    angleY = alpha * (angleY + angleY_gyro) + (1 - alpha) * angleY_accel;

//    Serial.print("AccelAngle: ");
//    Serial.println(round(angleY_accel));
//    Serial.print("GyroAngle: ");
//    Serial.println(round(angleY_gyro));
    Serial.print("Filtered: ");
    Serial.println(round(angleY));

    delay(100);
}
