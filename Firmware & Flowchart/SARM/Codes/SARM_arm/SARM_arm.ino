#include <BluetoothSerial.h>
#include <ESP32Servo.h>

// ---------------------- BLUETOOTH -------------------------
BluetoothSerial SerialBT;

// ---------------------- SERVO OBJECTS ----------------------
Servo servoBase;
Servo servoShoulder;
Servo servoElbow;
Servo servoWrist;
Servo servoGripper;

// ---------------------- SERVO PINS -------------------------
#define PIN_BASE       26
#define PIN_SHOULDER   27
#define PIN_ELBOW      32
#define PIN_WRIST      23
#define PIN_GRIPPER    19

// ---------------------- TIMING & TUNING -------------------
int stepDelayDefault = 8;   // ms delay between interpolation steps (increase to slow down motion)
int minSteps = 20;          // minimum number of interpolation steps (prevents very few large jumps)
int safety_margin = 2;      // degrees of extra safety margin to avoid chassis contact

// ---------------------- ARM ANGLES -------------------------
// CONTRACTED STATE
int contracted_shoulder = 140;
int contracted_elbow    = 110;
int up_wrist            = 130;
int contracted_gripper  = 10;

// EXPANDED STATE
int contracted_base     = 90;
int expanded_shoulder   = 10;
int expanded_elbow      = 45;
int down_wrist          = 100;
int expanded_gripper    = 90;

// ---------------------- SETUP ------------------------------
void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_ARM");

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servoBase.attach(PIN_BASE, 500, 2400);
  servoShoulder.attach(PIN_SHOULDER, 500, 2400);
  servoElbow.attach(PIN_ELBOW, 500, 2400);
  servoWrist.attach(PIN_WRIST, 500, 2400);
  servoGripper.attach(PIN_GRIPPER, 500, 2400);

  // Start in CONTRACTED STATE
  servoBase.write(contracted_base);
  servoShoulder.write(contracted_shoulder);
  servoElbow.write(contracted_elbow);
  servoWrist.write(up_wrist);
  servoGripper.write(contracted_gripper);

  Serial.println("Manipulator Ready via USB & Bluetooth!");
  SerialBT.println("Manipulator Ready via Bluetooth!");

  Serial.println("Commands:\nB x\nG 0-90\nS 0 or 1");
  SerialBT.println("Commands:\nB x\nG 0-90\nS 0 or 1");
}

// -----------------------------------------------------------
//  INTERPOLATED SYNCHRONIZED MOVEMENT (both move in proportion)
// -----------------------------------------------------------
void smoothMoveSyncInterpolated(Servo &s1, int start1, int target1,
                                Servo &s2, int start2, int target2,
                                int stepDelay = -1) {
  if (stepDelay < 0) stepDelay = stepDelayDefault;

  int delta1 = target1 - start1;
  int delta2 = target2 - start2;

  int steps = max(abs(delta1), abs(delta2));
  steps = max(steps, minSteps); // ensure we have reasonable number of steps

  for (int i = 1; i <= steps; ++i) {
    float frac = (float)i / (float)steps; // progress 0..1

    // linear interpolation
    int pos1 = start1 + round(delta1 * frac);
    int pos2 = start2 + round(delta2 * frac);

    // Safety: ensure elbow never goes beyond the interpolated safe value
    // (This prevents elbow outrunning shoulder in contraction/expansion)
    // If s1 == shoulder and s2 == elbow this helps; if different usage, fine.
    // We add a small safety margin to be conservative.
    // NOTE: This logic assumes s1 is shoulder and s2 is elbow for contractions/expansions call.
    // If you call this for other pairs, it does no harm (simple interp).
    // We only enforce the constraint in situations where start/target imply contracted <-> expanded.
    // Compute expected elbow based on shoulder fraction of progress:
    int expected_elbow = start2 + round(delta2 * frac);

    // pos2 should not be "ahead" of expected_elbow by more than margin
    if ((delta2 > 0 && pos2 > expected_elbow + safety_margin) ||
        (delta2 < 0 && pos2 < expected_elbow - safety_margin)) {
      pos2 = expected_elbow + (delta2 > 0 ? safety_margin : -safety_margin);
    }

    // apply positions
    s1.write(constrain(pos1, 0, 180));
    s2.write(constrain(pos2, 0, 180));

    delay(stepDelay);
  }

  // ensure final exact targets
  s1.write(constrain(target1, 0, 180));
  s2.write(constrain(target2, 0, 180));
}

// -----------------------------------------------------------
//  INDIVIDUAL SMOOTH MOVEMENT (For Base and Gripper)
// -----------------------------------------------------------
void smoothMove(Servo &servo, int target, int stepDelay = -1) {
  if (stepDelay < 0) stepDelay = stepDelayDefault;

  int current = servo.read();
  int delta = target - current;
  int steps = max(abs(delta), minSteps);

  for (int i = 1; i <= steps; ++i) {
    float frac = (float)i / (float)steps;
    int pos = current + round(delta * frac);
    servo.write(constrain(pos, 0, 180));
    delay(stepDelayDefault);
  }
  servo.write(constrain(target, 0, 180));
}

// -----------------------------------------------------------
//  ARM STATE MOVEMENTS (use interpolated sync)
// -----------------------------------------------------------
void moveArmContracted() {
  int startShoulder = servoShoulder.read();
  int startElbow    = servoElbow.read();

  // Use interpolated sync: both move proportionally from current -> contracted
  smoothMoveSyncInterpolated(servoShoulder, startShoulder, contracted_shoulder,
                             servoElbow,    startElbow,    contracted_elbow,
                             stepDelayDefault);

  Serial.println("Moved to CONTRACTED state");
  SerialBT.println("Moved to CONTRACTED state");
}

void moveArmExpanded() {
  int startShoulder = servoShoulder.read();
  int startElbow    = servoElbow.read();

  smoothMoveSyncInterpolated(servoShoulder, startShoulder, expanded_shoulder,
                             servoElbow,    startElbow,    expanded_elbow,
                             stepDelayDefault);

  Serial.println("Moved to EXPANDED state");
  SerialBT.println("Moved to EXPANDED state");
}

// -----------------------------------------------------------
//  COMMAND PARSER
// -----------------------------------------------------------
void processCommand(String cmd) {
  cmd.trim();

  // BASE CONTROL: B <angle>
  if (cmd.startsWith("B")) {
    int angle = cmd.substring(2).toInt();
    angle = constrain(angle, 0, 180);

    servoBase.write(angle);

    Serial.printf("Base → %d°\n", angle);
    SerialBT.printf("Base → %d°\n", angle);
  }

  // Wrist: W <angle>
  else if (cmd.startsWith("W")) {
    int angle = cmd.substring(2).toInt();
    angle = constrain(angle, down_wrist, up_wrist);

    servoWrist.write(angle);

    Serial.printf("Wrist → %d°\n", angle);
    SerialBT.printf("Wrist → %d°\n", angle);
  }

  // Gripper: G <angle>
  else if (cmd.startsWith("G")) {
    int angle = cmd.substring(2).toInt();
    angle = constrain(angle, 0, 80);

    servoGripper.write(angle);

    Serial.printf("Gripper → %d°\n", angle);
    SerialBT.printf("Gripper → %d°\n", angle);
  }

  // ARM STATE: S 0 or S 1
  else if (cmd.startsWith("S")) {
    int st = cmd.substring(2).toInt();

    if (st == 0) moveArmContracted();
    else if (st == 1) moveArmExpanded();
  }

  else {
    Serial.println("Invalid Command!");
    SerialBT.println("Invalid Command!");
  }
}

// -----------------------------------------------------------
//  MAIN LOOP
// -----------------------------------------------------------
void loop() {

  // USB input
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    processCommand(input);
  }

  // Bluetooth input
  if (SerialBT.available()) {
    String input = SerialBT.readStringUntil('\n');
    processCommand(input);
  }
}
