# MeArm Inverse kinematics
Author: **Jake Briscoe** University of Bath

**Objective**
This project implements inverse kinematics to control a MeArm robotic arm.  The goal is to move the arm’s end-effector precisely to a 
desired position (x y,z) from user input by converting the maths into programming and using triangle inequalities to confirm that the range 
is possible.

**Plan**
Implement functional Inverse Kinematic control using serial monitor to test and verify
Once verify, apply Inverse Kinematic theory into code and verify physically with a ruler.

**Measurements**
Trigonometry is the foundation of inverse kinematic theory so measurements are needed to be taken

Base Height (link1): 50mm
Upper Arm (link2): 80mm
Forearm (link3): 80mm
End Effector (link4): 40mm

**Safety Prerequisites**
It was observed during testing that the servos couldn't support the weight of full extension and there were physical limits of the angles to prevent self-collisions. These were tested by increasing the servo angle for each joint in isolation one degree at a time until failure which yielded the experimental values as shown.

For elbow: 10 to 140 deg
For shoulder: 40 to 180 deg
For Base: 0 to 180 deg

**Maths Required**
<img width="3024" height="4032" alt="IMG_3399" src="https://github.com/user-attachments/assets/ca0511a1-7b6c-4d7a-b7e3-a8bbe89a3dd2" />

