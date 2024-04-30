#include <ezButton.h>
#include <Servo.h>
#include <Wire.h>
#include <stdlib.h>

const int sabertoothPin1 = 9;
ezButton limitSwitch(7);
int input = 1500;

//Servo sb1;

void setup() {
  Serial.begin(9600); 
  limitSwitch.setDebounceTime(50);
  Wire.begin();

  //sb1.attach(sabertoothPin1);
}

void loop() {
  limitSwitch.loop();

  if (Serial.available() > 0){
    input = Serial.parseInt();
  }

  Serial.println(input);
  
  int state = limitSwitch.getState();

  if(state == LOW){
    //sb1.writeMicroseconds(1500);
    Serial.println("Pressed");
  }
  else{
    //sb1.writeMicroseconds(input);
  }
  

  delay(20);
  
}
