#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// ----------------------------
// MOTOR PIN ASSIGNMENTS
// ----------------------------

// L298N #1
int M1_PWM = 19;
int M1_IN1 = 21;
int M1_IN2 = 3;

int M2_PWM = 23;
int M2_IN1 = 1;
int M2_IN2 = 22;

// L298N #2
int M3_PWM = 0;
int M3_IN1 = 4;
int M3_IN2 = 16;

int M4_PWM = 18;
int M4_IN1 = 17;
int M4_IN2 = 5;

// Speed
int speedVal = 128;   // 0–255 for ESP32 LEDC

// ----------------------------
// helper function
// ----------------------------
void setMotor(int in1, int in2, int pwmPin, int direction, int spd)
{
  digitalWrite(in1, direction == 1 ? HIGH : LOW);
  digitalWrite(in2, direction == 1 ? LOW : HIGH);
  ledcWrite(pwmPin, spd);
}

// ----------------------------
// MOVEMENT FUNCTIONS
// ----------------------------

void forward()
{
  setMotor(M1_IN1, M1_IN2, M1_PWM, 1, speedVal);
  setMotor(M2_IN1, M2_IN2, M2_PWM, 1, speedVal);
  setMotor(M3_IN1, M3_IN2, M3_PWM, 1, speedVal);
  setMotor(M4_IN1, M4_IN2, M4_PWM, 1, speedVal);
   Serial.println("Forward");
}

void backward()
{
  setMotor(M1_IN1, M1_IN2, M1_PWM, 0, speedVal);
  setMotor(M2_IN1, M2_IN2, M2_PWM, 0, speedVal);
  setMotor(M3_IN1, M3_IN2, M3_PWM, 0, speedVal);
  setMotor(M4_IN1, M4_IN2, M4_PWM, 0, speedVal);
     Serial.println("Backward");

}

void left()   // strafe left
{
  setMotor(M1_IN1, M1_IN2, M1_PWM, 0, speedVal);
  setMotor(M2_IN1, M2_IN2, M2_PWM, 1, speedVal);
  setMotor(M3_IN1, M3_IN2, M3_PWM, 1, speedVal);
  setMotor(M4_IN1, M4_IN2, M4_PWM, 0, speedVal);
     Serial.println("Left");

}

void right()  // strafe right
{
  setMotor(M1_IN1, M1_IN2, M1_PWM, 1, speedVal);
  setMotor(M2_IN1, M2_IN2, M2_PWM, 0, speedVal);
  setMotor(M3_IN1, M3_IN2, M3_PWM, 0, speedVal);
  setMotor(M4_IN1, M4_IN2, M4_PWM, 1, speedVal);
     Serial.println("Right");

}

void rotateLeft()
{
  setMotor(M1_IN1, M1_IN2, M1_PWM, 0, speedVal);
  setMotor(M2_IN1, M2_IN2, M2_PWM, 1, speedVal);
  setMotor(M3_IN1, M3_IN2, M3_PWM, 0, speedVal);
  setMotor(M4_IN1, M4_IN2, M4_PWM, 1, speedVal);
     Serial.println("Rotateleft");

}

void rotateRight()
{
  setMotor(M1_IN1, M1_IN2, M1_PWM, 1, speedVal);
  setMotor(M2_IN1, M2_IN2, M2_PWM, 0, speedVal);
  setMotor(M3_IN1, M3_IN2, M3_PWM, 1, speedVal);
  setMotor(M4_IN1, M4_IN2, M4_PWM, 0, speedVal);
     Serial.println("RotateRight");

}

void stopAll()
{
  ledcWrite(M1_PWM, 0);
  ledcWrite(M2_PWM, 0);
  ledcWrite(M3_PWM, 0);
  ledcWrite(M4_PWM, 0);
     Serial.println("Stop");

}

// ----------------------------
// SETUP
// ----------------------------
void setup()
{
  Serial.begin(115200);
  SerialBT.begin("SARM_BASE");

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Direction pins
  pinMode(M1_IN1, OUTPUT); pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT); pinMode(M2_IN2, OUTPUT);
  pinMode(M3_IN1, OUTPUT); pinMode(M3_IN2, OUTPUT);
  pinMode(M4_IN1, OUTPUT); pinMode(M4_IN2, OUTPUT);
 
  ledcAttach(M1_PWM, 1000, 8);
  ledcAttach(M2_PWM, 1000, 8);
  ledcAttach(M3_PWM, 1000, 8);
  ledcAttach(M4_PWM, 1000, 8);

  stopAll();
}

// ----------------------------
// MAIN LOOP
// ----------------------------
void loop() {
  
  if (SerialBT.available())
  {
    String cmd = SerialBT.readStringUntil('\n');
    cmd.trim();

    if (cmd == "F") forward();
    else if (cmd == "B") backward();
    else if (cmd == "L") left();
    else if (cmd == "R") right();
    else if (cmd == "S") stopAll();
    else if (cmd == "CL") rotateLeft();
    else if (cmd == "CR") rotateRight();
  }
  delay(10);
}