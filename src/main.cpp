#include <Arduino.h>

#include "ServoEasing.hpp"

int switch_counter = 0;
uint32_t switch_pushed_then;

#define ENABLE_EASE_CUBIC
#define ENABLE_EASE_PRECISION
#define oneRotation 24800

int desiredRotation = 0;
int currentRotation = 0;

int shoulderPin = 4;
int elbowPin = 16;
int wristPin = 17;
int onPin = 25;

int directionPin = 5;
int stepPin = 18;
int enablePin = 19;

int homePosition[] = {180, 160, 35}; // shoulder, elbow, wrist

ServoEasing shoulder;
ServoEasing elbow;
ServoEasing wrist;

int pot_1 = 26;
int pot_2 = 27;
int pot_3 = 14;
int pot_4 = 12;

void setup() {
    Serial.begin(115200);
    shoulder.attach(shoulderPin, homePosition[0]);
    elbow.attach(elbowPin, homePosition[1]);
    wrist.attach(wristPin, homePosition[2]);
    setSpeedForAllServos(40); // 40 degrees per second

    shoulder.setEasingType(EASE_CUBIC_IN_OUT);
    elbow.setEasingType(EASE_CUBIC_IN_OUT);
    wrist.setEasingType(EASE_CUBIC_IN_OUT);


    pinMode(directionPin, OUTPUT);
    pinMode(stepPin, OUTPUT);
    pinMode(enablePin, OUTPUT);
    digitalWrite(directionPin, HIGH);

    pinMode(pot_1, INPUT);
    pinMode(pot_2, INPUT);
    pinMode(pot_3, INPUT);
    pinMode(pot_4, INPUT);

    pinMode(onPin, INPUT_PULLUP);
    

    delay(500); // Wait for servo to reach start position.
}
void goToAngle(int shoulderAngle, int elbowAngle, int wristAngle){
    setIntegerDegreeForAllServos(3, shoulderAngle, elbowAngle, wristAngle); // (number of servos, shoulder, elbow, wrist)
    setEaseToForAllServosSynchronizeAndWaitForAllServosToStop(); // blocks until all servos reach target position
}

void goHome(){
    goToAngle(homePosition[0], homePosition[1], homePosition[2]); // shoulder, elbow, wrist
}

void rotate(float rotations){
    digitalWrite(enablePin, LOW);
    for (int n = 0; n < (oneRotation*rotations); n++){
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(150);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(150);
    }
    digitalWrite(enablePin, HIGH);
}






void loop() 
{
    if (digitalRead(onPin) == LOW)
    {
        switch_pushed_then = millis();
        switch_counter++;
    }

    else
    {
        switch_counter = 0;
    }

   if (switch_counter > 10000)
    {
        goToAngle(150, 118, 28); // shoulder, elbow, wrist
        // delay(1000);    
        goToAngle(111, 64, 82); // shoulder, elbow, wrist
        // delay(1000);    
        goToAngle(63, 13, 27); // shoulder, elbow, wrist
        // delay(1000);    
        goToAngle(57, 0, 23); // shoulder, elbow, wrist
        // delay(1000);
        goToAngle(91, 76, 64); // shoulder, elbow, wrist  
        // delay(1000);    
        goToAngle(57, 0, 23); // shoulder, elbow, wrist
        delay(1000);
        goToAngle(63, 13, 27); // shoulder, elbow, wrist
        // delay(1000);     
//         Shoudler: 156 Elbow: 118 Wrist: 28
//         Shoudler: 111 Elbow: 64 Wrist: 82
//         Shoudler: 63 Elbow: 13 Wrist: 27
//         Shoudler: 57 Elbow: 0 Wrist: 23
//         Shoudler: 91 Elbow: 76 Wrist: 64
        goHome();
    }

    //Serial.println(millis()-switch_pushed_then);
    if (millis()-switch_pushed_then > 2000)
    {
        switch_counter = 0;
    }
    // Serial.print("pot 1 :");
    // Serial.println(analogRead(pot_1));
    // Serial.print("pot 2 :");
    // Serial.println(analogRead(pot_2));
    // Serial.print("pot 3 :");
    // Serial.println(analogRead(pot_3));

    // int servo1Pos = map(analogRead(pot_1), 0, 4095, 0, 180);
    // int servo2Pos = map(analogRead(pot_2), 0, 4095, 0, 180);
    // int servo3Pos = map(analogRead(pot_3), 0, 4095, 0, 180);

    // shoulder.write(servo1Pos);
    // elbow.write(servo2Pos);
    // wrist.write(servo3Pos);

    // if (digitalRead(onPin) == LOW){
    //     Serial.println("hi");
    //     Serial.println("Shoudler: " + String(servo1Pos) + " Elbow: " + String(servo2Pos) + " Wrist: " + String(servo3Pos));
    // }

    // shoulder.write(servo1Pos);
    // elbow.write(servo2Pos);
    // wrist.write(servo3Pos);



}


