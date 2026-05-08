#include <Servo.h>
#include <math.h>

Servo hip;
Servo knee;

// Unit: mm
const float l1 = 143.0; // Femur length
const float l2 = 125.0; // Tibia length

const float r2d = 57.2957795; // Rad to deg

float timeStep = 0.0;
const float stepSpeed = 0.15; // Gait speed
const float ellipseWidth = 30.0;  // Step length
const float ellipseHeight = 20.0; // How high foot lifts
const float startX = 0.0; // Center X
const float startY = -70.0; // Center Y

void setup() {
  hip.attach(3);
  knee.attach(5);
}

void loop() {
  float targetX = startX + (ellipseWidth * cos(timeStep));
  float targetY = startY + (ellipseHeight * sin(timeStep));
  
  timeStep += stepSpeed;
  if(timeStep > 2*PI) timeStep = 0;

  // IK
  float L = sqrt(targetX * targetX + targetY * targetY);
  
  // Knee angle
  float A = acos((sq(l1) + sq(l2) - sq(L)) / (2 * l1 * l2));
  // Hip angle
  float B = acos((sq(l1) + sq(L) - sq(l2)) / (2 * l1 * L)) + atan2(targetX, -targetY);
  
  float kneeAngle = A * r2d;
  float hipAngle = B * r2d;

  hip.write(180 - (hipAngle+15)); 
  knee.write(kneeAngle + 90);
  
  delay(10);
}
