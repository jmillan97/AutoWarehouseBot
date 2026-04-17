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

void setup() {
  int allPins[] = {M1_DIR, M1_PWM, M2_DIR, M2_PWM, M3_DIR, M3_PWM, M4_DIR, M4_PWM};
  for(int i = 0; i < 8; i++) pinMode(allPins[i], OUTPUT);

  Serial.begin(9600);
  Serial.println("ACK: SYSTEM_READY");
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
      // 2. Handle Movement Commands
      else if (command == "w") driveAllForward(robotSpeed);
      else if (command == "s") driveAllBackward(robotSpeed);
      else if (command == "d") driveAllRight(robotSpeed);
      else if (command == "a") driveAllLeft(robotSpeed);
      else if (command == "q") rotateCounterclockwise(robotSpeed);
      else if (command == "e") rotateClockwise(robotSpeed);
      else if (command == "stop") stopAll();
    }
  }

  // Safety Watchdog
  if (millis() - lastCmd > SAFETY_TIMEOUT) {
    stopAll();
  }
}

// --- Movement Functions ---

void driveAllForward(int s) {
  setMotor(M1_DIR, M1_PWM, s, LOW);
  setMotor(M3_DIR, M3_PWM, s, LOW);
  setMotor(M2_DIR, M2_PWM, s, LOW);
  setMotor(M4_DIR, M4_PWM, s, LOW);
}

void driveAllBackward(int s) {
  setMotor(M1_DIR, M1_PWM, s, HIGH);
  setMotor(M3_DIR, M3_PWM, s, HIGH);
  setMotor(M2_DIR, M2_PWM, s, HIGH);
  setMotor(M4_DIR, M4_PWM, s, HIGH);
}

void driveAllRight(int s) {
  setMotor(M1_DIR, M1_PWM, s, LOW);
  setMotor(M4_DIR, M4_PWM, s, LOW);
  setMotor(M2_DIR, M2_PWM, s, HIGH);
  setMotor(M3_DIR, M3_PWM, s, HIGH);
}

void driveAllLeft(int s) {
  setMotor(M1_DIR, M1_PWM, s, HIGH);
  setMotor(M4_DIR, M4_PWM, s, HIGH);
  setMotor(M2_DIR, M2_PWM, s, LOW);
  setMotor(M3_DIR, M3_PWM, s, LOW);
}

void rotateClockwise(int s) {
  setMotor(M1_DIR, M1_PWM, s, HIGH);
  setMotor(M3_DIR, M3_PWM, s, HIGH);
  setMotor(M2_DIR, M2_PWM, s, LOW);
  setMotor(M4_DIR, M4_PWM, s, LOW);
}

void rotateCounterclockwise(int s) {
  setMotor(M1_DIR, M1_PWM, s, LOW);
  setMotor(M3_DIR, M3_PWM, s, LOW);
  setMotor(M2_DIR, M2_PWM, s, HIGH);
  setMotor(M4_DIR, M4_PWM, s, HIGH);
}

void setMotor(int dirPin, int pwmPin, int s, bool logic) {
  digitalWrite(dirPin, logic);
  analogWrite(pwmPin, constrain(abs(s), 0, 255));
}

void stopAll() {
  analogWrite(M1_PWM, 0); analogWrite(M2_PWM, 0);
  analogWrite(M3_PWM, 0); analogWrite(M4_PWM, 0);
}
