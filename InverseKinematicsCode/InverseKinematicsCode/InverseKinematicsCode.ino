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
const float link34 = link3 + link4; //elbow to gripper tip

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


void inverseKinematics(float x, float y, float z, float &baseRad, float &shoulderRad, float &elbowRad) {

  //Calculate values to help model from 3D to 2D
  float z2 = z - link1; ///acounts for the height of the base from ground not included in trig
  float r = sqrt(x*x + y*y);
  float d = sqrt(r*r + (z2)*(z2)); //z taken from joint not ground

  //Triangle inequality to check whether position is valid
  if (d <= (link2 + link34) && d >= fabs(link2 - link34)) {
    Serial.println("Target is reachable!");
  } else {
    Serial.println("Target is out of reach!");
  } 

  //Calculate base angle by modelling in 2D
  baseRad = atan2(y,x);

  //Calculate elbow angle by modelling in 2D
  float cosElbowRad, cosShoulderRad;
  
  cosElbowRad = constrain((link2*link2 + link34*link34 - d*d)/(2*link2*link34),-1.0f,1.0f); //constrained to prevent noise from making it slightly > 1
  elbowRad = PI - acos(cosElbowRad); // cosine rule of 2D model and subtract by pi to get it other direction

  //Calculate shoulder angle

  cosShoulderRad = constrain((link2*link2 + d*d - link34*link34)/(2*link2*d),-1.0f,1.0f); 
  shoulderRad = atan2(z2,r) - acos(cosShoulderRad);  //Ensured that angles are the same direction by subtracting

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

    Serial.println("Input 3 distances for x, y, z respectively in format 'x y z'");

}

void loop() {
 if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); //Gets rid of the spacet to make it easier to process

    //Split the string into its three constituent values
    int sp1 = input.indexOf(' ');
    int sp2 = input.indexOf(' ', sp1 + 1);
    if (sp1 < 0 || sp2 < 0) {
      Serial.println("Format error. Use: x y z");
      return;
    }
    float x = input.substring(0, sp1).toFloat();
    float y = input.substring(sp1 + 1, sp2).toFloat();
    float z = input.substring(sp2 + 1).toFloat();

    //Check if angles are in mechanically safe ranges before starting
    if (baseDeg >= baseMin && baseDeg <= baseMax &&
      shoulderDeg >= shoulderMin && shoulderDeg <= shoulderMax &&
      elbowDeg >= elbowMin && elbowDeg <= elbowMax) {

        float baseRad, shoulderRad, elbowRad;
        inverseKinematics(x, y, z, baseRad, shoulderRad, elbowRad); // uses your function as-is

        float baseDeg = baseRad * 360.0/(2*PI); //Convert to degrees
        float shoulderDeg = shoulderRad * 360.0/(2*PI);
        float elbowDeg = elbowRad * 360.0/(2*PI);

        int baseCmd     = constrain((int)round(baseDeg),     baseMin,     baseMax);   
        int shoulderCmd = constrain((int)round(shoulderDeg), shoulderMin, shoulderMax); 
        int elbowCmd    = constrain((int)round(elbowDeg),    elbowMin,    elbowMax); 

        base.write(baseCmd);        
        shoulder.write(shoulderCmd);    
        elbow.write(elbowCmd);  

        Serial.print("IK rad  B/S/E: ");
        Serial.print(baseRad, 4); Serial.print(" / ");
        Serial.print(shoulderRad, 4); Serial.print(" / ");
        Serial.println(elbowRad, 4);

        Serial.print("Cmd deg B/S/E: ");
        Serial.print(baseCmd); Serial.print(" / ");
        Serial.print(shoulderCmd); Serial.print(" / ");
        Serial.println(elbowCmd);

    } else {
      // At least one joint is out of range
      Serial.println("Move skipped: one or more joints out of range.");
    }
}