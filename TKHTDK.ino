const int ENC_L = 2;
const int ENC_R = 3;

const int ENA = 5;
const int IN1 = 7;
const int IN2 = 8;

const int ENB = 6;
const int IN3 = 9;
const int IN4 = 10;

const int PPR = 20;

const float SETPOINT_RPM = 100.0;

// Thời gian lấy mẫu
const unsigned long SAMPLE_MS = 500;
const float Ts = SAMPLE_MS / 1000.0;

// PI giảm nhẹ để tránh kéo PWM quá mạnh
const float Kp_L = 0.30;
const float Ki_L = 0.35;

const float Kp_R = 0.30;
const float Ki_R = 0.35;

// PWM nền
// Bánh phải yếu hơn nên cho base phải cao hơn
const int BASE_PWM_L = 75;
const int BASE_PWM_R = 90;

const int PWM_MIN = 0;
const int PWM_MAX = 255;

// Giới hạn tích phân
const float I_LIMIT = 80.0;

// Lọc EMA
const float ALPHA = 0.35;

// Phase 1: chạy full PWM để test tốc độ thực tế
const unsigned long PHASE1_MS = 5000;
const int PHASE1_PWM = 255;

volatile long pulseL = 0;
volatile long pulseR = 0;

long lastPulseL = 0;
long lastPulseR = 0;

float rpmL_filtered = 0;
float rpmR_filtered = 0;

float integralL = 0;
float integralR = 0;

unsigned long lastSample = 0;
unsigned long phase2Start = 0;

bool phase2Started = false;
bool firstPhase2Sample = true;

void isrL() {
  pulseL++;
}

void isrR() {
  pulseR++;
}

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

void resetMeasureAndPI() {
  noInterrupts();
  pulseL = 0;
  pulseR = 0;
  interrupts();

  lastPulseL = 0;
  lastPulseR = 0;

  rpmL_filtered = 0;
  rpmR_filtered = 0;

  integralL = 0;
  integralR = 0;
}

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
  Serial.println("        DUAL MOTOR PI SPEED CONTROL - STABLE VERSION");
  Serial.println("==============================================================");
  Serial.println("Target RPM : 100");
  Serial.println("Sample Time: 500 ms");
  Serial.println("Left  PI: Kp = 0.30, Ki = 0.35");
  Serial.println("Right PI: Kp = 0.30, Ki = 0.35");
  Serial.println("Base PWM L/R: 75 / 90");
  Serial.println("Phase 1: PWM = 255 for 5 seconds");
  Serial.println("Phase 2: lower to BASE PWM first, then PI control");
  Serial.println("NOTE: Remove ENA/ENB jumpers if using Arduino PWM pins D5/D6");
  Serial.println("--------------------------------------------------------------");

  delay(2000);

  // ==========================================================
  // PHASE 1: FULL PWM 255, MEASURE RPM
  // ==========================================================

  Serial.println();
  Serial.println("==============================================================");
  Serial.println("PHASE 1 START: FULL PWM TEST");
  Serial.println("Motor PWM = 255, measuring raw RPM");
  Serial.println("==============================================================");
  Serial.println("PHASE | Time(ms) | Raw_L | Raw_R | PWM_L | PWM_R | Pulse_L | Pulse_R");
  Serial.println("--------------------------------------------------------------");

  resetMeasureAndPI();

  long phase1LastPulseL = 0;
  long phase1LastPulseR = 0;

  unsigned long phase1Start = millis();
  unsigned long phase1LastSample = phase1Start;

  setMotorLeft(PHASE1_PWM);
  setMotorRight(PHASE1_PWM);

  while (millis() - phase1Start < PHASE1_MS) {
    unsigned long now = millis();

    if (now - phase1LastSample >= SAMPLE_MS) {
      noInterrupts();
      long currentPulseL = pulseL;
      long currentPulseR = pulseR;
      interrupts();

      long dPulseL = currentPulseL - phase1LastPulseL;
      long dPulseR = currentPulseR - phase1LastPulseR;

      float rpmL_raw = (dPulseL / (float)PPR) / Ts * 60.0;
      float rpmR_raw = (dPulseR / (float)PPR) / Ts * 60.0;

      Serial.print("P1");
      Serial.print(" | Time=");
      Serial.print(now - phase1Start);
      Serial.print(" ms");

      Serial.print(" | Raw_L=");
      Serial.print(rpmL_raw, 1);

      Serial.print(" | Raw_R=");
      Serial.print(rpmR_raw, 1);

      Serial.print(" | PWM_L=");
      Serial.print(PHASE1_PWM);

      Serial.print(" | PWM_R=");
      Serial.print(PHASE1_PWM);

      Serial.print(" | Pulse_L=");
      Serial.print(currentPulseL);

      Serial.print(" | Pulse_R=");
      Serial.println(currentPulseR);

      phase1LastPulseL = currentPulseL;
      phase1LastPulseR = currentPulseR;

      phase1LastSample = now;
    }
  }

  // ==========================================================
  // CHUYỂN PHA: KHÔNG STOP HẲN, NHƯNG HẠ VỀ PWM NỀN
  // ==========================================================

  setMotorLeft(BASE_PWM_L);
  setMotorRight(BASE_PWM_R);

  Serial.println("--------------------------------------------------------------");
  Serial.println("PHASE 1 FINISHED");
  Serial.println("Lowering motors to BASE PWM before PHASE 2 PI control");
  Serial.println("--------------------------------------------------------------");

  // ==========================================================
  // RESET BEFORE PHASE 2
  // ==========================================================

  resetMeasureAndPI();

  lastSample = millis();
  phase2Start = lastSample;
  phase2Started = true;
  firstPhase2Sample = true;

  Serial.println();
  Serial.println("==============================================================");
  Serial.println("PHASE 2 START: PI SPEED CONTROL");
  Serial.println("Motors are first lowered to BASE PWM. PI then takes over.");
  Serial.println("==============================================================");
  Serial.println("PHASE | Time(ms) | Raw_L | Raw_R | Filt_L | Filt_R | PWM_L | PWM_R | Pulse_L | Pulse_R | Err_L | Err_R");
  Serial.println("--------------------------------------------------------------");
}

void loop() {
  if (!phase2Started) return;

  unsigned long now = millis();

  if (now - lastSample >= SAMPLE_MS) {
    noInterrupts();
    long currentPulseL = pulseL;
    long currentPulseR = pulseR;
    interrupts();

    long dPulseL = currentPulseL - lastPulseL;
    long dPulseR = currentPulseR - lastPulseR;

    float rpmL_raw = (dPulseL / (float)PPR) / Ts * 60.0;
    float rpmR_raw = (dPulseR / (float)PPR) / Ts * 60.0;

    // Mẫu đầu tiên phase 2: khởi tạo filter bằng raw luôn.
    // Không để filtered bắt đầu từ 0, vì sẽ làm PI tưởng RPM quá thấp.
    if (firstPhase2Sample) {
      rpmL_filtered = rpmL_raw;
      rpmR_filtered = rpmR_raw;
      firstPhase2Sample = false;
    } else {
      rpmL_filtered = ALPHA * rpmL_raw + (1.0 - ALPHA) * rpmL_filtered;
      rpmR_filtered = ALPHA * rpmR_raw + (1.0 - ALPHA) * rpmR_filtered;
    }

    float errorL = SETPOINT_RPM - rpmL_filtered;
    float errorR = SETPOINT_RPM - rpmR_filtered;

    // Cập nhật tích phân rồi kẹp lại.
    // Không dùng anti-windup quá gắt để tránh bánh bị tụt chết.
    float candidateIntegralL = integralL + errorL * Ts;
    float candidateIntegralR = integralR + errorR * Ts;

    integralL = constrain(candidateIntegralL, -I_LIMIT, I_LIMIT);
    integralR = constrain(candidateIntegralR, -I_LIMIT, I_LIMIT);

    float controlL = Kp_L * errorL + Ki_L * integralL;
    float controlR = Kp_R * errorR + Ki_R * integralR;

    int pwmL = BASE_PWM_L + controlL;
    int pwmR = BASE_PWM_R + controlR;

    pwmL = constrain(pwmL, PWM_MIN, PWM_MAX);
    pwmR = constrain(pwmR, PWM_MIN, PWM_MAX);

    setMotorLeft(pwmL);
    setMotorRight(pwmR);

    Serial.print("P2");
    Serial.print(" | Time=");
    Serial.print(now - phase2Start);
    Serial.print(" ms");

    Serial.print(" | Raw_L=");
    Serial.print(rpmL_raw, 1);

    Serial.print(" | Raw_R=");
    Serial.print(rpmR_raw, 1);

    Serial.print(" | Filt_L=");
    Serial.print(rpmL_filtered, 1);

    Serial.print(" | Filt_R=");
    Serial.print(rpmR_filtered, 1);

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

    lastSample = now;
  }
}