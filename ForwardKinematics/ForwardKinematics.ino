#include <Arduino.h>
#include <ESP32Servo.h>

//Pins number on Arduino
const int basePin = 4;
const int shoulderPin = 6;
const int elbowPin = 5;

Servo base, shoulder, elbow;

//Measured lengths between links
const float link1 = 50.0;  //base to shoulder
const float link2 = 80.0;  //shoulder to elbow
const float link3 = 80.0;  //elbow to wrist
const float link4 = 40.0;  //wrist to gripper tip

//Servo offsets for neutral position found experimentally
const float offsetBase = 90;
const float offsetShoulder = 90;
const float offsetElbow = 90;

//Safe angle limits (tested manually)
const int baseMin = 0;
const int baseMax = 180;
const int shoulderMin = 40;
const int shoulderMax = 180;
const int elbowMin = 10;
const int elbowMax = 140;

//Maths for forward kinematics
void forwardKinematics(float baseRad, float shoulderRad, float elbowRad, float &x, float &y, float &z) {
  float r = link2 * cos(shoulderRad) + (link3 + link4) * cos(shoulderRad + elbowRad);
  x = r * cos(baseRad);
  y = r * sin(baseRad);
  z = link1 + link2 * sin(shoulderRad) + (link3 + link4) * sin(shoulderRad + elbowRad);
}

void setup() {
  Serial.begin(115200);

  //Attach servos and check success
  if (!base.attach(basePin)) {
    Serial.println("Error: Failed to attach base servo");
  }
  if (!shoulder.attach(shoulderPin)) {
    Serial.println("Error: Failed to attach shoulder servo");
  }
  if (!elbow.attach(elbowPin)) {
    Serial.println("Error: Failed to attach elbow servo");
  }

  Serial.println("Input 3 angles for base, shoulder, elbow respectively in format 'x y z'");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); //Gets rid of the spacet to make it easier to process

    //Split the string into its three constituent values
    int baseDeg = input.substring(0, input.indexOf(' ')).toInt();
    int shoulderDeg = input.substring(input.indexOf(' ') + 1, input.lastIndexOf(' ')).toInt();
    int elbowDeg = input.substring(input.lastIndexOf(' ') + 1).toInt();

    //Check angle ranges are within the measured safe level
    if (baseDeg < baseMin || baseDeg > baseMax) {
      Serial.println("Error: Base angle out of range.");
      return;
    }
    if (shoulderDeg < shoulderMin || shoulderDeg > shoulderMax) {
      Serial.println("Error: Shoulder angle out of range.");
      return;
    }
    if (elbowDeg < elbowMin || elbowDeg > elbowMax) {
      Serial.println("Error: Elbow angle out of range.");
      return;
    }

    base.write(baseDeg);
    shoulder.write(shoulderDeg);
    elbow.write(elbowDeg);

    float baseRad = radians(baseDeg - offsetBase);
    float shoulderRad = radians(shoulderDeg - offsetShoulder);
    float elbowRad = radians(elbowDeg - offsetElbow);

    float x, y, z;
    forwardKinematics(baseRad, shoulderRad, elbowRad, x, y, z);

    //Print out the information
    Serial.print("Angles in degrees: ");
    Serial.print(baseDeg); Serial.print(", ");
    Serial.print(shoulderDeg); Serial.print(", ");
    Serial.println(elbowDeg);

    Serial.print("Position in mm: ");
    Serial.print(x); 
    Serial.print(", ");
    Serial.print(y); 
    Serial.print(", ");
    Serial.println(z);
    Serial.println();
  }
}