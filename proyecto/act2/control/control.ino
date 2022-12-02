// Imorttar librerias
#include "DualVNH5019MotorShield.h"
DualVNH5019MotorShield md;

// Pines
#define encoder0PinA 19
#define encoder0PinB 18
#define encoder1PinA 20
#define encoder1PinB 21

// Variables
#define RPM 31250

int out0 = 0;
int out1 = 0;

int ref0 = 150;
int ref1 = 150;

double e0;
double e0_old = 0;
double e0_old2 = 0;
double e1;
double e1_old = 0;
double e1_old2 = 0;

// Control PID
double kp = 0.0004;
double ki = 0.000003;
double kd = 0.00005;


// Variables encoder
volatile long encoder0Pos = 0;
volatile long encoder1Pos = 0;

long newposition0;
long oldposition0 = 0;
long newposition1;
long oldposition1 = 0;

double vel0;
double vel1;

unsigned long newtime;
unsigned long oldtime = 0;

// Configuracion interrupciones
void doEncoder0A() {
    if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
        encoder0Pos++;
    } else {
        encoder0Pos--;
    }
}

void doEncoder0B() {
    if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
        encoder0Pos--;
    } else {
        encoder0Pos++;
    }
}

void doEncoder1A() {
    if (digitalRead(encoder1PinA) == digitalRead(encoder1PinB)) {
        encoder1Pos++;
    } else {
        encoder1Pos--;
    }
}

void doEncoder1B() {
    if (digitalRead(encoder1PinA) == digitalRead(encoder1PinB)) {
        encoder1Pos--;
    } else {
        encoder1Pos++;
    }
}

// Setup
void setup() {
    // Configurar MotorShield
    md.init();

    // Configurar encoders
    pinMode(encoder0PinA, INPUT);
    digitalWrite(encoder0PinA, HIGH);
    pinMode(encoder0PinB, INPUT);
    digitalWrite(encoder0PinB, HIGH);
    pinMode(encoder1PinA, INPUT);
    digitalWrite(encoder1PinA, HIGH);
    pinMode(encoder1PinB, INPUT);
    digitalWrite(encoder1PinB, HIGH);

    attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoder0A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoder0PinB), doEncoder0B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoder1PinA), doEncoder1A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoder1PinB), doEncoder1B, CHANGE);

    // Serial port
    Serial.begin(115200);
}

// Loop
void loop() {
    newtime = micros();

    newposition0 = encoder0Pos;
    newposition1 = encoder1Pos;

    vel0 = (double)(newposition0 - oldposition0) * RPM / (newtime - oldtime);
    vel1 = (double)(newposition1 - oldposition1) * RPM / (newtime - oldtime);

    oldposition0 = newposition0;
    oldposition1 = newposition1;

    e0_old2 = e0_old;
    e1_old2 = e1_old;

    e0_old = e0;
    e1_old = e1;

    e0 = ref0 + vel0;
    e1 = ref1 - vel1;

    // Control PID velocidad
    double dt = (newtime - oldtime);

    double k0 = kp + ki * dt + kd / dt;
    double k1 = -kp - 2 * kd / dt;
    double k2 = kd / dt;

    out0 = (out0 + k0 * e0 + k1 * e0_old + k2 * e0_old2);
    out1 = (out1 + k0 * e1 + k1 * e1_old + k2 * e1_old2);

    // Motor Voltage
    md.setM1Speed(-out0);
    md.setM2Speed(out1);

    Serial.print("Ref ");
    Serial.print(" || ");
    Serial.print(vel0);
    Serial.print(" | ");
    Serial.println(vel1);

    delay(100);
    oldtime = newtime;
}