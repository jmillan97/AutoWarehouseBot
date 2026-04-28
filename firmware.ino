#include <EnableInterrupt.h>

/*
 * ROBOT FIRMWARE (3-27-2026 - Speed Steering)
 * Dynamic Speed + Hold-to-Move + ACK Feedback
 */

const int M1_DIR = 4;  const int M1_PWM = 3;
const int M2_DIR = 12; const int M2_PWM = 11;
const int M3_DIR = 8;  const int M3_PWM = 5;
const int M4_DIR = 7;  const int M4_PWM = 6;

// --- DYNAMIC SPEED ---
volatile int robotSpeed = 100; // Initialize at 100

unsigned long lastCmd = 0;
const int SAFETY_TIMEOUT = 1000;

volatile long ticksFR = 0; // Front-Right (M1)
volatile long ticksRL = 0; // Rear-Left (M4)

const int ENC_FR_A = A0; // Interrupt 0
const int ENC_FR_B = 9;
const int ENC_RL_A = A1; // Interrupt 1
const int ENC_RL_B = 10;

/*
B - Green (Front = 9, Back = 10)
A - Orange (Front = A0, Back = A1)
Sensor VCC - Yellow
Sensor GND - White
*/ 

unsigned long prevTime = 0;
const int UPDATE_INTERVAL = 50; // 20Hz feedback loop

// Distance move model (must match wheel_odometry assumptions reasonably well)
const float WHEEL_RADIUS_M = 0.04f;
const float WHEEL_SEPARATION_M = 0.21f;
const float ENCODER_CPR = 2.0f;
const float GEAR_RATIO = 30.0f;
const float EFFECTIVE_CPR = ENCODER_CPR * GEAR_RATIO;
const float METERS_PER_TICK = (2.0f * PI * WHEEL_RADIUS_M) / EFFECTIVE_CPR;
const unsigned long MOVE_MAX_RUNTIME_MS = 15000;

volatile bool moveActive = false;
volatile int moveMode = 0; // 0=none, 1=linear, 2=rotate
volatile int moveDirection = 0; // linear: +1 forward/-1 backward, rotate: +1 CCW/-1 CW
volatile long moveTargetTicks = 0; // target based on avg absolute encoder delta
volatile long moveStartFR = 0;
volatile long moveStartRL = 0;
volatile unsigned long moveStartMs = 0;

void setup() {
  int allPins[] = {M1_DIR, M1_PWM, M2_DIR, M2_PWM, M3_DIR, M3_PWM, M4_DIR, M4_PWM};
  for(int i = 0; i < 8; i++) pinMode(allPins[i], OUTPUT);

  pinMode(ENC_RL_A, INPUT_PULLUP);
  pinMode(ENC_RL_B, INPUT_PULLUP);
  pinMode(ENC_FR_A, INPUT_PULLUP);
  pinMode(ENC_FR_B, INPUT_PULLUP);

  enableInterrupt(ENC_RL_A, handleRL, RISING);
  enableInterrupt(ENC_FR_A, handleFR, RISING);

  Serial.begin(115200);
  Serial.println("ACK: SYSTEM_READY");
  lastCmd = millis();
}

long averageDistanceTicksFromStart() {
  long dfr = labs(ticksFR - moveStartFR);
  long drl = labs(ticksRL - moveStartRL);
  return (dfr + drl) / 2;
}

void beginDistanceMove(long mm) {
  if (mm == 0) {
    stopAll();
    moveActive = false;
    Serial.println("ACK: MOVE_DONE");
    return;
  }

  float meters = fabs(mm) / 1000.0f;
  long targetTicks = (long)round(meters / METERS_PER_TICK);
  if (targetTicks < 1) targetTicks = 1;

  moveDirection = (mm > 0) ? 1 : -1;
  moveTargetTicks = targetTicks;
  moveStartFR = ticksFR;
  moveStartRL = ticksRL;
  moveStartMs = millis();
  moveActive = true;
  moveMode = 1;

  // Use at least a moderate PWM to avoid motor deadband stall.
  int driveSpeed = constrain(robotSpeed, 90, 255);
  if (moveDirection > 0) driveAllForward(driveSpeed);
  else driveAllBackward(driveSpeed);

  Serial.print("ACK: MOVE_START ");
  Serial.print(mm);
  Serial.print("mm ");
  Serial.print("targetTicks=");
  Serial.println(moveTargetTicks);
}

void beginRotateMove(long deg) {
  if (deg == 0) {
    stopAll();
    moveActive = false;
    moveMode = 0;
    Serial.println("ACK: ROTATE_DONE");
    return;
  }

  float theta = fabs(deg) * PI / 180.0f;
  float wheelArcMeters = (WHEEL_SEPARATION_M / 2.0f) * theta;
  long targetTicks = (long)round(wheelArcMeters / METERS_PER_TICK);
  if (targetTicks < 1) targetTicks = 1;

  moveDirection = (deg > 0) ? 1 : -1;
  moveTargetTicks = targetTicks;
  moveStartFR = ticksFR;
  moveStartRL = ticksRL;
  moveStartMs = millis();
  moveActive = true;
  moveMode = 2;

  int driveSpeed = constrain(robotSpeed, 90, 255);
  if (moveDirection > 0) rotateCounterclockwise(driveSpeed);
  else rotateClockwise(driveSpeed);

  Serial.print("ACK: ROTATE_START ");
  Serial.print(deg);
  Serial.print("deg ");
  Serial.print("targetTicks=");
  Serial.println(moveTargetTicks);
}

void updateDistanceMove() {
  if (!moveActive) return;

  long traveled = averageDistanceTicksFromStart();
  if (traveled >= moveTargetTicks) {
    stopAll();
    moveActive = false;
    if (moveMode == 2) Serial.println("ACK: ROTATE_DONE");
    else Serial.println("ACK: MOVE_DONE");
    moveMode = 0;
    return;
  }

  if (millis() - moveStartMs > MOVE_MAX_RUNTIME_MS) {
    stopAll();
    moveActive = false;
    if (moveMode == 2) Serial.println("ACK: ROTATE_TIMEOUT");
    else Serial.println("ACK: MOVE_TIMEOUT");
    moveMode = 0;
    return;
  }

  // Keep watchdog alive during an active distance move.
  lastCmd = millis();
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();

    if (command.length() > 0) {
      lastCmd = millis();

      // 1. Handle Speed Updates (Format: "speed:150")
      if (command.startsWith("speed:")) {
        int newSpeed = command.substring(6).toInt();
        robotSpeed = constrain(newSpeed, 0, 255);
        Serial.print("ACK: SPEED ");
        Serial.println(robotSpeed);
      }
      // 1b. Exact distance move in mm (signed): "move_mm:500" or "move_mm:-250"
      else if (command.startsWith("move_mm:")) {
        long mm = command.substring(8).toInt();
        beginDistanceMove(mm);
      }
      // 1c. Exact rotate in signed degrees: "rotate_deg:90" / "rotate_deg:-45"
      else if (command.startsWith("rotate_deg:")) {
        long deg = command.substring(11).toInt();
        beginRotateMove(deg);
      }
      // 1d. Independent side drive: "drive_lr:<left_pwm>,<right_pwm>"
      // Signed PWM range is -255..255. Positive drives that side forward.
      else if (command.startsWith("drive_lr:")) {
        moveActive = false;
        moveMode = 0;
        String payload = command.substring(9);
        int comma = payload.indexOf(',');
        if (comma > 0) {
          int leftPwm = payload.substring(0, comma).toInt();
          int rightPwm = payload.substring(comma + 1).toInt();
          driveLR(leftPwm, rightPwm);
          Serial.print("ACK: DRIVE_LR ");
          Serial.print(constrain(leftPwm, -255, 255));
          Serial.print(",");
          Serial.println(constrain(rightPwm, -255, 255));
        } else {
          Serial.println("ACK: DRIVE_LR_BAD_FORMAT");
        }
      }
      // 2. Legacy manual commands (kept for manual bench testing)
      else if (command == "w") { moveActive = false; moveMode = 0; driveAllForward(robotSpeed); }
      else if (command == "s") { moveActive = false; moveMode = 0; driveAllBackward(robotSpeed); }
      else if (command == "d") { moveActive = false; moveMode = 0; driveAllRight(robotSpeed); }
      else if (command == "a") { moveActive = false; moveMode = 0; driveAllLeft(robotSpeed); }
      else if (command == "q") { moveActive = false; moveMode = 0; rotateCounterclockwise(robotSpeed); }
      else if (command == "e") { moveActive = false; moveMode = 0; rotateClockwise(robotSpeed); }
      else if (command == "x") { moveActive = false; moveMode = 0; stopAll(); }
      else if (command == "r") ticksReset();
    }
  }

  updateDistanceMove();

  if(millis() - prevTime >= UPDATE_INTERVAL) {
   prevTime = millis();

   Serial.print("E:");
   Serial.print(ticksFR);
   Serial.print(",");
   Serial.print(ticksRL);
   Serial.print("\n");
  }

  // Safety Watchdog
  if (!moveActive && millis() - lastCmd > SAFETY_TIMEOUT) {
    stopAll();
  }
}

// --- Movement Functions ---

void driveAllForward(int s) {
  setMotor(M1_DIR, M1_PWM, s, HIGH);
  setMotor(M3_DIR, M3_PWM, s, HIGH);
  setMotor(M2_DIR, M2_PWM, s, HIGH);
  setMotor(M4_DIR, M4_PWM, s, HIGH);
}

void driveAllBackward(int s) {
  setMotor(M1_DIR, M1_PWM, s, LOW);
  setMotor(M3_DIR, M3_PWM, s, LOW);
  setMotor(M2_DIR, M2_PWM, s, LOW);
  setMotor(M4_DIR, M4_PWM, s, LOW);
}

void driveAllRight(int s) {
  setMotor(M1_DIR, M1_PWM, s, HIGH);
  setMotor(M4_DIR, M4_PWM, s, HIGH);
  setMotor(M2_DIR, M2_PWM, s, LOW);
  setMotor(M3_DIR, M3_PWM, s, LOW);
}

void driveAllLeft(int s) {
  setMotor(M1_DIR, M1_PWM, s, LOW);
  setMotor(M4_DIR, M4_PWM, s, LOW);
  setMotor(M2_DIR, M2_PWM, s, HIGH);
  setMotor(M3_DIR, M3_PWM, s, HIGH);
}

void rotateClockwise(int s) {
  setMotor(M1_DIR, M1_PWM, s, LOW);
  setMotor(M3_DIR, M3_PWM, s, LOW);
  setMotor(M2_DIR, M2_PWM, s, HIGH);
  setMotor(M4_DIR, M4_PWM, s, HIGH);
}

void rotateCounterclockwise(int s) {
  setMotor(M1_DIR, M1_PWM, s, HIGH);
  setMotor(M3_DIR, M3_PWM, s, HIGH);
  setMotor(M2_DIR, M2_PWM, s, LOW);
  setMotor(M4_DIR, M4_PWM, s, LOW);
}

void driveRightSide(int pwm) {
  bool forward = pwm >= 0;
  setMotor(M1_DIR, M1_PWM, pwm, forward ? HIGH : LOW);
  setMotor(M3_DIR, M3_PWM, pwm, forward ? HIGH : LOW);
}

void driveLeftSide(int pwm) {
  bool forward = pwm >= 0;
  setMotor(M2_DIR, M2_PWM, pwm, forward ? HIGH : LOW);
  setMotor(M4_DIR, M4_PWM, pwm, forward ? HIGH : LOW);
}

void driveLR(int leftPwm, int rightPwm) {
  leftPwm = constrain(leftPwm, -255, 255);
  rightPwm = constrain(rightPwm, -255, 255);
  driveLeftSide(leftPwm);
  driveRightSide(rightPwm);
}

void setMotor(int dirPin, int pwmPin, int s, bool logic) {
  digitalWrite(dirPin, logic);
  analogWrite(pwmPin, constrain(abs(s), 0, 255));
}

void stopAll() {
  analogWrite(M1_PWM, 0); analogWrite(M2_PWM, 0);
  analogWrite(M3_PWM, 0); analogWrite(M4_PWM, 0);
}

void handleFR() {
  if(digitalRead(ENC_FR_B) == LOW) ticksFR--;
  else ticksFR++;
}

void handleRL() {
  if(digitalRead(ENC_RL_B) == LOW) ticksRL++;
  else ticksRL--;
}

void ticksReset() {
  ticksRL = 0;
  ticksFR = 0;
  Serial.println("\nCounters Reset");
}
