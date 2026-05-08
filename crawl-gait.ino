// Crawl gait (3 legs on ground so statically stable)
#include <Servo.h>
#include <math.h>

Servo hipFL;
Servo kneeFL;
Servo hipFR;
Servo kneeFR;
Servo hipBL;
Servo kneeBL;
Servo hipBR;
Servo kneeBR;

#define joy_translate 18
#define joy_turn 19

float hipOffset = 15;
float kneeOffset = 90;

// Unit: mm
const float l1 = 143.0; // Femur length
const float l2 = 125.0; // Tibia length

const float r2d = 57.2957795; // Rad to deg

float timeStep = 0.0;
const float stepSpeed = 0.25; // Gait speed
const float ellipseWidth = 30.0;  // Step length
const float ellipseHeight = 20.0; // How high foot lifts
const float startX = 0.0; // Center X
const float startY = -70.0; // Center Y below hip

volatile uint32_t throttleStart = 0;
volatile int throttlePulse = 1500;
volatile uint32_t turnStart = 0;
volatile int turnPulse = 1500;

float filteredThrottle = 1500;
float filteredTurn = 1500;

bool isIdle = true;

const float PHASE_FL = 0.0;
const float PHASE_FR = 0.5 * PI;
const float PHASE_BR = 1.0 * PI;
const float PHASE_BL = 1.5 * PI;

void setup() {
  hipBR.attach(2);
  kneeBR.attach(3);
  hipFR.attach(4);
  kneeFR.attach(5);
  hipFL.attach(6);
  kneeFL.attach(7);
  hipBL.attach(8);
  kneeBL.attach(9);

  pinMode(joy_translate, INPUT);
  pinMode(joy_turn, INPUT);

  attachInterrupt(digitalPinToInterrupt(joy_translate), throttleISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(joy_turn), steeringISR, CHANGE);
}

void loop() {
  noInterrupts();
  int rawThrottle = throttlePulse;
  int rawTurn = turnPulse;
  interrupts();

  filteredThrottle = 0.9 * filteredThrottle + 0.1 * rawThrottle;
  int t = (int)filteredThrottle;

  filteredTurn = 0.9 * filteredTurn + 0.1 * rawTurn;
  int turn = (int)filteredTurn;

  bool forward = (t > 1550);
  bool backward = (t < 1450);
  bool turnLeft = (turn > 1550);
  bool turnRight = (turn < 1450);

  if (forward || backward || turnLeft || turnRight) {
    isIdle = false;

    timeStep += stepSpeed;
    if(timeStep > 2*PI) timeStep -= 2*PI;

    if (forward) {
      moveLeg(hipFL, kneeFL, timeStep + PHASE_FL, true);
      moveLeg(hipBR, kneeBR, timeStep + PHASE_BR, true);
      moveLeg(hipFR, kneeFR, timeStep + PHASE_FR, true);
      moveLeg(hipBL, kneeBL, timeStep + PHASE_BL, true);
    } else if (backward) {
      moveLeg(hipFL, kneeFL, timeStep + PHASE_FL, false);
      moveLeg(hipBR, kneeBR, timeStep + PHASE_BR, false);
      moveLeg(hipFR, kneeFR, timeStep + PHASE_FR, false);
      moveLeg(hipBL, kneeBL, timeStep + PHASE_BL, false);
    } else if (turnLeft) {
      moveLeg(hipFL, kneeFL, timeStep + PHASE_FL, false);
      moveLeg(hipBR, kneeBR, timeStep + PHASE_BR, true);
      moveLeg(hipFR, kneeFR, timeStep + PHASE_FR, true);
      moveLeg(hipBL, kneeBL, timeStep + PHASE_BL, false);
    } else if (turnRight) {
      moveLeg(hipFL, kneeFL, timeStep + PHASE_FL, true);
      moveLeg(hipBR, kneeBR, timeStep + PHASE_BR, false);
      moveLeg(hipFR, kneeFR, timeStep + PHASE_FR, false);
      moveLeg(hipBL, kneeBL, timeStep + PHASE_BL, true);
    }
  }
  else {
    if (!isIdle) {
      setAllServos(90);
      isIdle = true;
    }
  }
  delay(10);
}

void moveLeg(Servo& hip, Servo& knee, float phase, bool mirror) {
  float phaseNorm = fmod(phase, 2 * PI);

  float x, y;

  // Swing: 16.7% of cycle, stance: 83.3% of cycle
  if (phaseNorm < PI/3) {
    float t = phaseNorm / (PI/3);

    x = startX + ellipseWidth * (t - 0.5);
    //x = startX + ellipseWidth * 0.5 * cos(t * PI); // More natural?
    y = startY + ellipseHeight * sin(t * PI);

  } else {
    float t = (phaseNorm - PI/3) / (2*PI-PI/3);

    // Backstep
    x = startX + ellipseWidth * (0.5 - t);

    //y = startY;
    y = startY + 0.05 * ellipseHeight * sin(t * PI); // Adds compliance
  }

  if (mirror) x = -x;

  // IK
  float L = sqrt(x*x + y*y);

  float val = (sq(l1) + sq(l2) - sq(L)) / (2 * l1 * l2);
  val = constrain(val, -1, 1);
  float A = acos(val);

  float B = acos((sq(l1) + sq(L) - sq(l2)) / (2 * l1 * L)) + atan2(x, -y);

  float kneeAngle = A * r2d;
  float hipAngle  = B * r2d;

  hip.write(180 - (hipAngle + hipOffset));
  knee.write(kneeAngle + kneeOffset);
}

void throttleISR() {
  if (digitalRead(joy_translate) == HIGH) {
    throttleStart = micros();
  } else {
    throttlePulse = micros() - throttleStart;
  }
}

void steeringISR() {
  if (digitalRead(joy_turn) == HIGH) {
    turnStart = micros();
  } else {
    turnPulse = micros() - turnStart;
  }
}

void setAllServos(int angle) {
  hipFL.write(angle);
  kneeFL.write(angle);
  hipFR.write(angle);
  kneeFR.write(angle);
  hipBL.write(angle);
  kneeBL.write(angle);
  hipBR.write(angle);
  kneeBR.write(angle);
}
