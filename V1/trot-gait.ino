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

// Offsets for fwd movement
float hipOffsetF = 45; // Better push: 50
float kneeOffsetF = 50;

// Offsets for bwd movement
float hipOffsetB = 35;
float kneeOffsetB = 60;

// Offsets for turning
float hipOffsetT = 40;
float kneeOffsetT = 50;

float currHipOffset;
float currKneeOffset;

// Unit: mm
const float l1 = 143.0; // Femur length
const float l2 = 125.0; // Tibia length

const float r2d = 57.2957795; // Rad to deg

float timeStep = 0.0;
const float stepSpeed = 1.8; // Gait speed
const float ellipseWidth = 85.0; // Step length
const float ellipseHeight = 120.0; // How high foot lifts
const float startX = 0.0; // Center X
const float startY = -60.0; // Center Y below hip

volatile uint32_t throttleStart = 0;
volatile int throttlePulse = 1500;
volatile uint32_t turnStart = 0;
volatile int turnPulse = 1500;

float filteredThrottle = 1500;
float filteredTurn = 1500;

bool isIdle = true;

const float PHASE_FL = 0.0;
const float PHASE_FR = PI;
const float PHASE_BR = 0.0;
const float PHASE_BL = PI;

const float swingPhase = PI;

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

static uint32_t prevTime = micros();

void loop() {
  uint32_t now = micros();
  float dt = (now - prevTime) / 1000000.0f;
  prevTime = now;

  if (dt > 0.02f) dt = 0.02f;
  
  noInterrupts();
  int rawThrottle = throttlePulse;
  int rawTurn = turnPulse;
  interrupts();

  filteredThrottle = 0.9 * filteredThrottle + 0.1 * rawThrottle;
  int t = (int)filteredThrottle;

  filteredTurn = 0.9 * filteredTurn + 0.1 * rawTurn;
  int turn = (int)filteredTurn;

  bool forward = (t < 1450);
  bool backward = (t > 1550);
  bool turnLeft = (turn < 1450);
  bool turnRight = (turn > 1550);

  if (forward || backward || turnLeft || turnRight) {
    isIdle = false;

    timeStep += stepSpeed * TWO_PI * dt;

    if (timeStep >= TWO_PI) timeStep -= TWO_PI;

    // Direction multipliers
    // 1 = fwd
    // -1 = bwd
    int dirFL = 0;
    int dirFR = 0;
    int dirBL = 0;
    int dirBR = 0;

    if (forward) {
        dirFL =  1;
        dirFR =  1;
        dirBL =  1;
        dirBR =  1;

    } else if (backward) {
        dirFL = -1;
        dirFR = -1;
        dirBL = -1;
        dirBR = -1;

    } else if (turnLeft) {
        dirFL = -1;
        dirBL = -1;
        dirFR =  1;
        dirBR =  1;

    } else if (turnRight) {
        dirFL =  1;
        dirBL =  1;
        dirFR = -1;
        dirBR = -1;
    }

    if (forward) {
      currHipOffset = hipOffsetF;
      currKneeOffset = kneeOffsetF;
    } else if (backward) {
        currHipOffset = hipOffsetB;
        currKneeOffset = kneeOffsetB;
    } else {
      currHipOffset = hipOffsetT;
      currKneeOffset = kneeOffsetT;
    }

    moveLeg(hipFL, kneeFL, timeStep + PHASE_FL, dirFL, currHipOffset, currKneeOffset);
    moveLeg(hipFR, kneeFR, timeStep + PHASE_FR, dirFR, currHipOffset, currKneeOffset);
    moveLeg(hipBL, kneeBL, timeStep + PHASE_BL, dirBL, currHipOffset, currKneeOffset);
    moveLeg(hipBR, kneeBR, timeStep + PHASE_BR, dirBR, currHipOffset, currKneeOffset);
  }
  else {
    if (!isIdle) {
      setAllServos(70);
      isIdle = true;
    }
  }
}

void moveLeg(Servo& hip, Servo& knee, float phase, int direction, float hipOffset, float kneeOffset) {
  float phaseNorm = fmod(phase, 2 * PI);

  float x, y;

  if (phaseNorm < swingPhase) {
    float t = phaseNorm / swingPhase;

    x = startX - direction * ellipseWidth * (0.5 - t);
    y = startY - ellipseHeight * sin(t * PI);

  } else {
    float t = (phaseNorm - swingPhase) / (2*PI-swingPhase);

    x = startX - direction * (-ellipseWidth * 0.5 + ellipseWidth * (t * 0.9)); // reduce torque
    //y = startY + 10.0 * sin(t * PI);
    y = startY + sin(t*PI);
  }

  // IK (loc)
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
