// ================= PIN CONFIG =================

const int ENC_L = 2;
const int ENC_R = 3;

const int ENA = 5;
const int IN1 = 7;
const int IN2 = 8;

const int ENB = 6;
const int IN3 = 9;
const int IN4 = 10;

// ================= PARAMETERS =================

const int PPR = 20;

const float SETPOINT_RPM = 100.0;

const unsigned long SAMPLE_MS = 100;
const float Ts = SAMPLE_MS / 1000.0;

// PI parameters from MATLAB
const float Kp_R = 0.89395;
const float Ki_R = 31.185;
const float Kd_R = 0.0;

const float Kp_L = 0.65213;
const float Ki_L = 46.801;
const float Kd_L = 0.0;

// Feedforward PWM, estimated manually
int basePWM_L = 50;
int basePWM_R = 55;

// PWM limits
const int PWM_MIN = 0;
const int PWM_MAX = 255;

// Integral limit to reduce windup
const float I_LIMIT = 5.0;

// Startup boost
const unsigned long STARTUP_MAX_PWM_MS = 5000;
const int STARTUP_PWM = 255;

// ================= VARIABLES =================

volatile long pulseL = 0;
volatile long pulseR = 0;

long lastPulseL = 0;
long lastPulseR = 0;

float integralL = 0;
float integralR = 0;

float prevErrorL = 0;
float prevErrorR = 0;

unsigned long lastSample = 0;

bool piStarted = false;

// ================= INTERRUPT =================

void isrL() {
  pulseL++;
}

void isrR() {
  pulseR++;
}

// ================= MOTOR CONTROL =================

void setMotorLeft(int pwm) {
  pwm = constrain(pwm, PWM_MIN, PWM_MAX);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  analogWrite(ENA, pwm);
}

void setMotorRight(int pwm) {
  pwm = constrain(pwm, PWM_MIN, PWM_MAX);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENB, pwm);
}

void stopMotor() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// ================= RESET FUNCTION =================

void resetEncoderAndPI() {
  noInterrupts();
  pulseL = 0;
  pulseR = 0;
  interrupts();

  lastPulseL = 0;
  lastPulseR = 0;

  integralL = 0;
  integralR = 0;

  prevErrorL = 0;
  prevErrorR = 0;
}

// ================= SETUP =================

void setup() {
  Serial.begin(115200);

  pinMode(ENC_L, INPUT_PULLUP);
  pinMode(ENC_R, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L), isrL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R), isrR, RISING);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  stopMotor();

  Serial.println("==============================================================");
  Serial.println("        DUAL MOTOR PI SPEED CONTROL WITH STARTUP BOOST");
  Serial.println("==============================================================");
  Serial.println("Target RPM : 100");
  Serial.println("Sample Time: 100 ms");
  Serial.println("Startup   : PWM = 255 for 5 seconds");
  Serial.println();
  Serial.println("Left  PI: Kp = 0.65213, Ki = 46.801, Kd = 0");
  Serial.println("Right PI: Kp = 0.89395, Ki = 31.185, Kd = 0");
  Serial.println("==============================================================");
  Serial.println();

  delay(2000);

  // ==========================================================
  // PHASE 1: STARTUP BOOST PWM 255
  // ==========================================================

  Serial.println("==============================================================");
  Serial.println("PHASE 1 START: FULL PWM BOOST");
  Serial.println("Both motors running at PWM = 255 for 5 seconds");
  Serial.println("==============================================================");
  Serial.println("Format:");
  Serial.println("BOOST | Time(ms) | RPM_L | RPM_R | PWM_L | PWM_R | Pulse_L | Pulse_R");
  Serial.println("--------------------------------------------------------------");

  resetEncoderAndPI();

  long boostLastPulseL = 0;
  long boostLastPulseR = 0;

  unsigned long boostStart = millis();
  unsigned long lastBoostSample = boostStart;

  setMotorLeft(STARTUP_PWM);
  setMotorRight(STARTUP_PWM);

  while (millis() - boostStart < STARTUP_MAX_PWM_MS) {
    unsigned long now = millis();

    if (now - lastBoostSample >= SAMPLE_MS) {
      noInterrupts();
      long currentPulseL = pulseL;
      long currentPulseR = pulseR;
      interrupts();

      long dPulseL = currentPulseL - boostLastPulseL;
      long dPulseR = currentPulseR - boostLastPulseR;

      float rpmL = (dPulseL / (float)PPR) / Ts * 60.0;
      float rpmR = (dPulseR / (float)PPR) / Ts * 60.0;

      Serial.print("BOOST | Time=");
      Serial.print(now - boostStart);
      Serial.print(" ms");

      Serial.print(" | RPM_L=");
      Serial.print(rpmL, 1);

      Serial.print(" | RPM_R=");
      Serial.print(rpmR, 1);

      Serial.print(" | PWM_L=");
      Serial.print(STARTUP_PWM);

      Serial.print(" | PWM_R=");
      Serial.print(STARTUP_PWM);

      Serial.print(" | Pulse_L=");
      Serial.print(currentPulseL);

      Serial.print(" | Pulse_R=");
      Serial.println(currentPulseR);

      boostLastPulseL = currentPulseL;
      boostLastPulseR = currentPulseR;

      lastBoostSample = now;
    }
  }

  // ==========================================================
  // DIRECT SWITCH TO PHASE 2, DO NOT STOP MOTORS
  // ==========================================================

  Serial.println("--------------------------------------------------------------");
  Serial.println("PHASE 1 FINISHED");
  Serial.println("Switching directly to PI control without stopping motors");
  Serial.println("--------------------------------------------------------------");

  Serial.println();
  Serial.println("==============================================================");
  Serial.println("RESETTING ENCODER AND PI VARIABLES");
  Serial.println("Motors are still running, PI will take over immediately");
  Serial.println("==============================================================");

  resetEncoderAndPI();

  lastSample = millis();
  piStarted = true;

  // ==========================================================
  // PHASE 2: PI CONTROL
  // ==========================================================

  Serial.println();
  Serial.println("==============================================================");
  Serial.println("PHASE 2 START: PI SPEED CONTROL");
  Serial.println("==============================================================");
  Serial.println("Target RPM : 100");
  Serial.println("Format:");
  Serial.println("PI | Time(ms) | RPM_L | RPM_R | PWM_L | PWM_R | Pulse_L | Pulse_R | Err_L | Err_R");
  Serial.println("--------------------------------------------------------------");
}

// ================= LOOP =================

void loop() {
  if (!piStarted) return;

  unsigned long now = millis();

  if (now - lastSample >= SAMPLE_MS) {
    noInterrupts();
    long currentPulseL = pulseL;
    long currentPulseR = pulseR;
    interrupts();

    long dPulseL = currentPulseL - lastPulseL;
    long dPulseR = currentPulseR - lastPulseR;

    float rpmL = (dPulseL / (float)PPR) / Ts * 60.0;
    float rpmR = (dPulseR / (float)PPR) / Ts * 60.0;

    float errorL = SETPOINT_RPM - rpmL;
    float errorR = SETPOINT_RPM - rpmR;

    integralL += errorL * Ts;
    integralR += errorR * Ts;

    integralL = constrain(integralL, -I_LIMIT, I_LIMIT);
    integralR = constrain(integralR, -I_LIMIT, I_LIMIT);

    float derivativeL = (errorL - prevErrorL) / Ts;
    float derivativeR = (errorR - prevErrorR) / Ts;

    float controlL = Kp_L * errorL + Ki_L * integralL + Kd_L * derivativeL;
    float controlR = Kp_R * errorR + Ki_R * integralR + Kd_R * derivativeR;

    int pwmL = basePWM_L + controlL;
    int pwmR = basePWM_R + controlR;

    pwmL = constrain(pwmL, PWM_MIN, PWM_MAX);
    pwmR = constrain(pwmR, PWM_MIN, PWM_MAX);

    setMotorLeft(pwmL);
    setMotorRight(pwmR);

    Serial.print("PI | Time=");
    Serial.print(now);
    Serial.print(" ms");

    Serial.print(" | RPM_L=");
    Serial.print(rpmL, 1);

    Serial.print(" | RPM_R=");
    Serial.print(rpmR, 1);

    Serial.print(" | PWM_L=");
    Serial.print(pwmL);

    Serial.print(" | PWM_R=");
    Serial.print(pwmR);

    Serial.print(" | Pulse_L=");
    Serial.print(currentPulseL);

    Serial.print(" | Pulse_R=");
    Serial.print(currentPulseR);

    Serial.print(" | Err_L=");
    Serial.print(errorL, 1);

    Serial.print(" | Err_R=");
    Serial.println(errorR, 1);

    lastPulseL = currentPulseL;
    lastPulseR = currentPulseR;

    prevErrorL = errorL;
    prevErrorR = errorR;

    lastSample = now;
  }
}