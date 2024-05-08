#include <Wire.h>
#include <basicMPU6050.h>

basicMPU6050<> mpu;

float angleY = 0;
unsigned long previousTime = 0;
float deltaTime = 0;
float alpha = 0.98;

void setup() {
    Serial.begin(9600);
    Wire.begin();
    mpu.setup();
}

void loop() {
    unsigned long currentTime = millis();
    deltaTime = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;

    /*angleY += (mpu.gy() * (180 / PI)) * deltaTime;
    int roundedAngleY = round(angleY);*/

    float accelY = mpu.ay();
    float accelZ = mpu.az();
    
    float accelAngleY = accelY / sqrt(accelY * accelY + accelZ * accelZ);
    angleY = alpha * (angleY + mpu.gy() * (180 / PI) * deltaTime) + (1 - alpha) * accelAngleY;

    Serial.print("Angle Y: ");
    Serial.println(angleY);

    delay(100);
}
