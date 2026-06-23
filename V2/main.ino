#include <Servo.h>
#include <math.h>
#include <NewPing.h>
#include <LiquidCrystal.h>

LiquidCrystal lcd(39, 38, 40, 41, 42, 43);

#define trig_pin 2
#define echo_pin 3

NewPing sonar(trig_pin, echo_pin, 400);

Servo abFL;
Servo hipFL;
Servo kneeFL;

Servo abFR;
Servo hipFR;
Servo kneeFR;

Servo abBL;
Servo hipBL;
Servo kneeBL;

Servo abBR;
Servo hipBR;
Servo kneeBR;

#define joy_translate 18
#define joy_turn 19
#define joy_strafe 20
#define joy_mode 21

// Offsets for forwards and turning
float hipOffsetF = 50;
float kneeOffsetF = 10;

// Offsets for backwards
float hipOffsetB = 45;
float kneeOffsetB = 20;

// Offsets for strafing
float hipOffsetS = 40; // OLD: 45
float kneeOffsetS = 45; // OLD: 30

float currHipOffset;
float currKneeOffset;

// Unit: mm
const float l1 = 143.0; // Femur length
const float l2 = 130.0; // Tibia length, added bigger feet

const float r2d = 57.2957795; // Rad to deg

float timeStep = 0.0;
float stepSpeed; // Gait speed

float ellipseWidth; // Step length
float ellipseHeight; // How high foot lifts

const float startX = 0.0; // Center X
const float startZ = -150.0;

volatile uint32_t throttleStart = 0;
volatile int throttlePulse = 1500;

volatile uint32_t turnStart = 0;
volatile int turnPulse = 1500;

volatile uint32_t strafeStart = 0;
volatile int strafePulse = 1500;

volatile uint32_t modeStart = 0;
volatile int modePulse = 1000;

float filteredThrottle = 1500;
float filteredTurn = 1500;
float filteredStrafe = 1500;
float filteredMode = 1000;

bool isIdle = true;

float PHASE_FL;
float PHASE_FR;
float PHASE_BR;
float PHASE_BL;

const float swingPhase = PI;

int lastAbFL = 90;
int lastHipFL = 90;
int lastKneeFL = 90;

int lastAbFR = 90;
int lastHipFR = 90;
int lastKneeFR = 90;

int lastAbBL = 90;
int lastHipBL = 90;
int lastKneeBL = 90;

int lastAbBR = 90;
int lastHipBR = 90;
int lastKneeBR = 90;

void setup() {
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Casper V2! :D");
  lcd.setCursor(0, 1);
  lcd.print("By Jonathan Li");
  //Serial.begin(115200);
  abFL.attach(10);
  hipFL.attach(9);
  kneeFL.attach(8);

  abFR.attach(22);
  hipFR.attach(23);
  kneeFR.attach(24);

  abBR.attach(51);
  hipBR.attach(52);
  kneeBR.attach(53);

  abBL.attach(13);
  hipBL.attach(12);
  kneeBL.attach(11);

  pinMode(joy_translate, INPUT);
  pinMode(joy_turn, INPUT);
  pinMode(joy_strafe, INPUT);
  pinMode(joy_mode, INPUT);

  attachInterrupt(digitalPinToInterrupt(joy_translate), throttleISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(joy_turn), steeringISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(joy_strafe), strafeISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(joy_mode), modeISR, CHANGE);
  
}

static uint32_t prevTime = micros();
unsigned long obsModeTo = 0;

void loop() {
  //Serial.println(sonar.ping_cm());
  uint32_t now = micros();
  float dt = (now - prevTime) / 1000000.0f; // Time elapsed
  prevTime = now;

  if (dt > 0.02f) dt = 0.02f;
  
  noInterrupts();
  int rawThrottle = throttlePulse;
  int rawTurn = turnPulse;
  int rawStrafe = strafePulse;
  int rawMode = modePulse;
  interrupts();

  // LPFs for joysticks
  filteredThrottle = 0.9 * filteredThrottle + 0.1 * rawThrottle;
  int t = (int)filteredThrottle;
  float throttleMag = abs(t - 1500);
  throttleMag = max(0.0f, throttleMag - 50.0f);

  filteredTurn = 0.9 * filteredTurn + 0.1 * rawTurn;
  int turn = (int)filteredTurn;

  filteredStrafe = 0.9 * filteredStrafe + 0.1 * rawStrafe;
  int strafe = (int)filteredStrafe;
  float strafeMag = abs(strafe - 1500);
  strafeMag = max(0.0f, strafeMag - 150.0f);

  filteredMode = 0.9 * filteredMode + 0.1 * rawMode;
  int mode = (int)filteredMode;

  bool backward = (t < 1450);
  bool forward = (t > 1550);
  bool turnRight = (turn < 1400);
  bool turnLeft = (turn > 1600);
  bool strafeRight = (strafe > 1650); // Since my transmitter doesn't have self-centered throttle more deadband needed
  bool strafeLeft = (strafe < 1350);
  bool obsDetected = sonar.ping_cm() < 40;
  // bool poseMode = mode > 1500;

  if (obsDetected) {
    obsModeTo = millis() + 5000; // 5 sec time period to go over obstacle
  }

  bool obsMode = millis() < obsModeTo;

  if (forward || backward || turnLeft || turnRight || strafeLeft || strafeRight) {
    isIdle = false;

    // Speed logic
    if (forward || backward) {
      if (forward) {
        stepSpeed = 1.2f + (throttleMag / 450.0f) * 0.8f;
      } else {
        stepSpeed = 1.2f + (throttleMag / 450.0f) * 1.0f;
      }
    } else if (strafeLeft || strafeRight) {
      stepSpeed = 1.5f + (strafeMag / 350.0f) * 1.0f;
    }

    stepSpeed = constrain(stepSpeed, 1.2f, 2.5f);

    timeStep += stepSpeed * TWO_PI * dt;

    if (timeStep >= TWO_PI) timeStep -= TWO_PI;

    /* POSE MODE
    if (!poseMode) {
      timeStep += stepSpeed * TWO_PI * dt;
      if (timeStep >= TWO_PI) timeStep -= TWO_PI;
    }*/

    // Direction multipliers
    // +1 = forward gait
    // -1 = backward gait
    int dirFL = 0;
    int dirFR = 0;
    int dirBL = 0;
    int dirBR = 0;

    PHASE_FL = 0.0;
    PHASE_FR = PI;
    PHASE_BR = 0.0;
    PHASE_BL = PI;

    ellipseHeight = 90;
    ellipseWidth = 90;
    //stepSpeed = 1.2;

    float strideFL = 1;
    float strideFR = 1;
    float strideBL = 1;
    float strideBR = 1;

    float latFL = 0;
    float latFR = 0;
    float latBL = 0;
    float latBR = 0;

    int moveDir = 0;
    if (forward) moveDir = 1;
    if (backward) moveDir = -1;

    dirFL = dirFR = dirBL = dirBR = moveDir;

    //if (forward || backward) stepSpeed = 1.2;

    /*
    if (forward) {
      dirFL = 1;
      dirFR = 1;
      dirBL = 1;
      dirBR = 1;

    } else if (backward) {
        dirFL = -1;
        dirFR = -1;
        dirBL = -1;
        dirBR = -1;
    }

    // Arc steering
    if (turnLeft && forward) {
      stepSpeed = 2;
      dirFL = dirFR = dirBL = dirBR = 1;
      strideFL = 0.3;
      strideBL = 0.3;
      strideFR = 1.0;
      strideBR = 1.0;

    } else if (turnRight && forward) {
        stepSpeed = 2;
        dirFL = dirFR = dirBL = dirBR = 1;
        strideFL = 1.0;
        strideBL = 1.0;
        strideFR = 0.3;
        strideBR = 0.3;
    }

    if (turnLeft && backward) {
      stepSpeed = 2;
      dirFL = dirFR = dirBL = dirBR = -1;
      strideFL = 1.0;
      strideBL = 1.0;
      strideFR = 0.3;
      strideBR = 0.3;
    } else if (turnRight && backward) {
      stepSpeed = 2;
      dirFL = dirFR = dirBL = dirBR = -1;
      strideFL = 0.3;
      strideBL = 0.3;
      strideFR = 1.0;
      strideBR = 1.0;
    }*/

    if (moveDir != 0 && (turnLeft || turnRight)) {
      //stepSpeed = 2.0;

      float innerStride = 0.5;
      float outerStride = 1;

      bool leftInner;

      if (moveDir > 0) {
        // Forward
        leftInner = turnLeft;
      } else {
        // Backward (steering reverses)
        leftInner = turnRight;
      }

      strideFL = strideBL = leftInner ? innerStride : outerStride;
      strideFR = strideBR = leftInner ? outerStride : innerStride;
    }
    
    if (strafeLeft || strafeRight) {
      ellipseHeight = 30;
      //stepSpeed = 2.2;

      float strafeDir = strafeLeft ? -1.0f : 1.0f;

      latFL = strafeDir * 25.0 * sin(timeStep + PHASE_FL);
      latFR = strafeDir * 25.0 * sin(timeStep + PHASE_FR);
      latBL = strafeDir * 25.0 * sin(timeStep + PHASE_BL);
      latBR = strafeDir * 25.0 * sin(timeStep + PHASE_BR);
    }

    if (moveDir == 0 && (turnLeft || turnRight)) {
      //ellipseHeight = 20;
      stepSpeed = 2;

      float turnDir = turnLeft ? -1.0f : 1.0f;
      float amp = 30.0f;

      latFL = -turnDir * amp * sin(timeStep + PHASE_FL);
      latBL = turnDir * amp * sin(timeStep + PHASE_BL);
      latFR = -turnDir * amp * sin(timeStep + PHASE_FR);
      latBR = turnDir * amp * sin(timeStep + PHASE_BR);
    }

    if (obsMode) {
      ellipseHeight = 120;
      ellipseWidth = 30;
      stepSpeed = 0.4;

      PHASE_FL = 0.0;
      PHASE_FR = PI;
      PHASE_BR = 0.5 * PI;
      PHASE_BL = 1.5 * PI;
      //hipOffsetF = 52;
      //kneeOffsetF = 40;
    }

    /* FOR POSE MODE
    if (poseMode) {
      float yawPose = 40.0f;

      if (turnLeft) {
        latFL = yawPose;
        latFR = yawPose;
        latBL = -yawPose;
        latBR = -yawPose;
      } else if (turnRight) {
        latFL = -yawPose;
        latFR = -yawPose;
        latBL = yawPose;
        latBR = yawPose;
      }
    }*/

    if (forward || turnLeft || turnRight) {
      currHipOffset = hipOffsetF;
      currKneeOffset = kneeOffsetF;
    } else if (backward) {
      currHipOffset = hipOffsetB;
      currKneeOffset = kneeOffsetB;
    } else { // For strafing
      currHipOffset = hipOffsetS;
      currKneeOffset = kneeOffsetS;
    }

    moveLeg(abFL, hipFL, kneeFL, timeStep + PHASE_FL, dirFL, strideFL, latFL, currHipOffset, currKneeOffset);
    moveLeg(abFR, hipFR, kneeFR, timeStep + PHASE_FR, dirFR, strideFR, latFR, currHipOffset, currKneeOffset);
    moveLeg(abBL, hipBL, kneeBL, timeStep + PHASE_BL, dirBL, strideBL, latBL, currHipOffset, currKneeOffset);
    moveLeg(abBR, hipBR, kneeBR, timeStep + PHASE_BR, dirBR, strideBR, latBR, currHipOffset, currKneeOffset);

  }
  else {
    if (!isIdle) {
      setAllServos(90);
      isIdle = true;
    }
  }
}

void writeServoDeadband(Servo& s, int target, int& lastTarget, int deadband = 2) {
  if (abs(target - lastTarget) >= deadband) {
    s.write(target);
    lastTarget = target;
  }
}

void moveLeg(Servo& abd, Servo& hip, Servo& knee, float phase, int direction, float strideScale, float lateral, float hipOffset, float kneeOffset) {
  float phaseNorm = fmod(phase, 2 * PI);

  float x, z;

  if (phaseNorm < swingPhase) {
    float t = phaseNorm / swingPhase;

    x = startX - direction * ellipseWidth * strideScale * (0.5 - t);

    /*
    if (t < 0.5) {
      z = startZ + ellipseHeight * (t * 2.0);
    }
    else {
      z = startZ + ellipseHeight;
    }*/
    z = startZ - ellipseHeight * sin(t * PI);

  } else {
    float t = (phaseNorm - swingPhase) / (2*PI-swingPhase);

    x = startX - direction * (-ellipseWidth * 0.5 + ellipseWidth * t) * strideScale;
    //z = startZ - 15.0 * sin(t * PI); // NEEDED to generate friction
    z = startZ;
  }

  // Strafe foot target
  float y = lateral;

  // Abduction angle
  float abdAngle = atan2(y, -z) * r2d;

  // Project into leg plane
  float zProj = sqrt(y*y + z*z);

  // IK
  float L = sqrt(x*x + zProj * zProj);
  L = constrain(L, 10, l1 + l2 - 5);

  float val = (sq(l1) + sq(l2) - sq(L)) / (2 * l1 * l2);
  val = constrain(val, -1, 1);
  float A = acos(val);

  float B = acos((sq(l1) + sq(L) - sq(l2)) / (2 * l1 * L)) + atan2(x, zProj);

  float kneeAngle = A * r2d;
  float hipAngle  = B * r2d;

  int abCmd = 90 + abdAngle;
  int hipCmd = 180 - (hipAngle + hipOffset);
  int kneeCmd = kneeAngle + kneeOffset;

  int* lastAb;
  int* lastHip;
  int* lastKnee;

  if (&hip == &hipFL) {
    lastAb = &lastAbFL;
    lastHip = &lastHipFL;
    lastKnee = &lastKneeFL;
  }
  else if (&hip == &hipFR) {
    lastAb = &lastAbFR;
    lastHip = &lastHipFR;
    lastKnee = &lastKneeFR;
  }
  else if (&hip == &hipBL) {
    lastAb = &lastAbBL;
    lastHip = &lastHipBL;
    lastKnee = &lastKneeBL;
  }
  else {
    lastAb = &lastAbBR;
    lastHip = &lastHipBR;
    lastKnee = &lastKneeBR;
  }

  //abd.write(90 + abdAngle);

  writeServoDeadband(abd, abCmd, *lastAb, 2);
  writeServoDeadband(hip, hipCmd, *lastHip, 2);
  writeServoDeadband(knee, kneeCmd, *lastKnee, 2);

  //hip.write(180 - (hipAngle + hipOffset));
  //knee.write(kneeAngle + kneeOffset);
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

void strafeISR() {
  if (digitalRead(joy_strafe) == HIGH) {
    strafeStart = micros();
  } else {
    strafePulse = micros() - strafeStart;
  }
}

void modeISR() {
  if (digitalRead(joy_mode) == HIGH) {
    modeStart = micros();
  } else {
    modePulse = micros() - modeStart;
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

  abFL.write(90);
  abFR.write(90);
  abBL.write(90);
  abBR.write(90);
}
