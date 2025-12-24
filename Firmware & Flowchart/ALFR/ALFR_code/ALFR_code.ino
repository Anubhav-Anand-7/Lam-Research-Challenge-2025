// --- PIN DEFINITIONS ---
#define ENA 10
#define IN1 9
#define IN2 8
#define ENB 5
#define IN3 7
#define IN4 6

// Ultrasonic Pins
#define TRIG_PIN 4
#define ECHO_PIN 3  // must be interrupt pin INT1

// Sensor pins
int sensorPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
int sensorValues[8];

// PID variables
float Kp = 40.0, Ki = 0.01, Kd = 6.0;
float P, I, D, PIDvalue, previousError = 0;
float baseSpeed = 128;

// last turn
int lastTurn = 0;

// ultrasonic vars
volatile unsigned long echoStart = 0;
volatile unsigned long echoEnd   = 0;
volatile bool echoHigh = false;
volatile float distanceCM = 999;
volatile bool emergencyStop = false;

unsigned long lastTrig = 0;

// ===== MOTOR =====
void setMotor(int L, int R) {
    if (emergencyStop) {
        analogWrite(ENA, 0);
        analogWrite(ENB, 0);
        return;
    }

    // left
    if (L >= 0) { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
    else        { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  L = -L; }

    // right
    if (R >= 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
    else        { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); R = -R; }

    analogWrite(ENA, constrain(L, 0, 255));
    analogWrite(ENB, constrain(R, 0, 255));
}

// ===== SENSOR READ =====
float readPosition() {
    long num = 0, den = 0;
    int active = 0;

    for (int i = 0; i < 8; i++) {
        int v = analogRead(sensorPins[i]);
        sensorValues[i] = v;
        if (v < 100) v = 0;
        if (v > 200) active++;

        num += (long)v * i;
        den += v;
    }

    if (active == 0 || den == 0) return -1;
    return (float)num / den;
}

// ===== ULTRASONIC ISR =====
void echoISR() {
    // check rising or falling edge via PIND
    if (PIND & (1 << PIND3)) {
        // rising
        echoStart = micros();
    } else {
        // falling
        echoEnd = micros();
        unsigned long duration = echoEnd - echoStart;

        // convert to cm
        float d = duration * 0.0343 / 2.0;
        distanceCM = d;

        // emergency stop logic
        if (d > 1 && d < 20) emergencyStop = true;
        else if (d >= 20)    emergencyStop = false;
    }
}

void setup() {
    Serial.begin(9600);

    for (int i = 0; i < 8; i++) pinMode(sensorPins[i], INPUT);

    pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

    pinMode(TRIG_PIN, OUTPUT);
    digitalWrite(TRIG_PIN, LOW);

    pinMode(ECHO_PIN, INPUT);

    // Disable analog comparator (VERY IMPORTANT!!)
    ACSR = (1 << ACD);

    // Attach interrupt
    attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoISR, CHANGE);
}

void loop() {

    // send TRIG pulse every 60ms
    if (millis() - lastTrig > 60) {
        lastTrig = millis();
        digitalWrite(TRIG_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN, LOW);
    }

    // if stopped by sonar
    if (emergencyStop) {
        setMotor(0, 0);
        return;
    }

    // ===== LINE FOLLOWING =====
    float pos = readPosition();
    if (pos == -1) {
        if (lastTurn == -1) setMotor(-120, 120);
        else if (lastTurn == 1) setMotor(120, -120);
        else setMotor(0, 0);
        return;
    }

    float error = 3.5 - pos;
    P = error;
    D = error - previousError;
    PIDvalue = Kp * P + Kd * D;
    previousError = error;

    float absErr = abs(error);
    int dynBase = baseSpeed - (absErr * 20);
    if (dynBase < 60) dynBase = 60;

    int L = dynBase - PIDvalue;
    int R = dynBase + PIDvalue;

    L = constrain(L, -255, 255);
    R = constrain(R, -255, 255);

    setMotor(L, R);

    if (error > 0) lastTurn = -1;
    else if (error < 0) lastTurn = 1;
}