// ---------- Encoder Right ----------
volatile long encoderCount_R = 0;
const int encoderPinA_R = 19;
const int encoderPinB_R = 18;
const float wheelCircumferenceMM_R = 408.41;
const int pulsesPerRevolution_R = 300;

// ---------- Encoder Left ----------
volatile long encoderCount_L = 0;
const int encoderPinA_L = 2;
const int encoderPinB_L = 3;
const float wheelCircumferenceMM_L = 408.41;
const int pulsesPerRevolution_L = 300;

// ---------- Robot Parameters ----------
const float WHEEL_BASE = 415.0; // mm (ระยะห่างล้อซ้าย-ขวา)
float theta_rad = 0.0;

// ---------- Motor ----------
#define PWM_R 4
#define DIR_R 5
#define PWM_L 6
#define DIR_L 7
#define PWM_EXTRA 8
#define DIR_EXTRA 9
int extraSpeed = 250;
int maxSpeed = 35;
int turnSpeed = 20; 

// ---------- Timing ----------
unsigned long lastTime = 0;
long lastCount_L = 0;
long lastCount_R = 0;

void setup() {
  Serial.begin(115200);

  pinMode(encoderPinA_R, INPUT_PULLUP);
  pinMode(encoderPinB_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA_R), readEncoderRight, CHANGE);

  pinMode(encoderPinA_L, INPUT_PULLUP);
  pinMode(encoderPinB_L, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA_L), readEncoderLeft, CHANGE);

  pinMode(PWM_R, OUTPUT); pinMode(DIR_R, OUTPUT);
  pinMode(PWM_L, OUTPUT); pinMode(DIR_L, OUTPUT);
  analogWrite(PWM_R, 0); analogWrite(PWM_L, 0);

  pinMode(PWM_EXTRA, OUTPUT);
  pinMode(DIR_EXTRA, OUTPUT);
  analogWrite(PWM_EXTRA, 0);
  Serial.println("Ready for command: UP / DOWN / STOP");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
if (cmd == "FORWARD") {
  moveMotor(turnSpeed, turnSpeed, false, false);
}
else if (cmd == "BACKWARD") {
  moveMotor(maxSpeed, maxSpeed, true, false);
}
else if (cmd == "LEFT") {
	moveMotor(turnSpeed, turnSpeed, true, true);
}
else if (cmd == "RIGHT") {
  moveMotor(turnSpeed, turnSpeed, false, false);

}
else if (cmd == "UP") {
  Serial.println("⬆️ Going Up");
  digitalWrite(DIR_EXTRA, LOW);
  analogWrite(PWM_EXTRA, extraSpeed);
}
else if (cmd == "DOWN") {
  Serial.println("⬇️ Going Down");
  digitalWrite(DIR_EXTRA, HIGH);
  analogWrite(PWM_EXTRA, extraSpeed);
}
else if (cmd == "STOP") {
  Serial.println("🛑 Stop All Motors");
  // หยุดทุกมอเตอร์
  moveMotor(0, 0, true, true);
  analogWrite(PWM_EXTRA, 0);
}

  }

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  if (dt >= 0.1) {
    long deltaL = encoderCount_L - lastCount_L;
    long deltaR = encoderCount_R - lastCount_R;
    lastCount_L = encoderCount_L;
    lastCount_R = encoderCount_R;

    float distL = (deltaL * wheelCircumferenceMM_L) / pulsesPerRevolution_L / 1000.0; // m
    float distR = (deltaR * wheelCircumferenceMM_R) / pulsesPerRevolution_R / 1000.0;

    float speedL = distL / dt;  // m/s
    float speedR = distR / dt;

    float dtheta = (distR - distL) / (WHEEL_BASE / 1000.0);  // rad
    theta_rad += dtheta;

    // จำกัดให้อยู่ในช่วง -π ถึง π
    if (theta_rad > M_PI) theta_rad -= 2 * M_PI;
    if (theta_rad < -M_PI) theta_rad += 2 * M_PI;

    float theta_deg = theta_rad * 180.0 / M_PI;

    Serial.print("ENC,");
    Serial.print(speedL, 4); Serial.print(",");
    Serial.print(speedR, 4); Serial.print(",");
    Serial.println(theta_deg, 2);

    lastTime = now;
  }
}

void moveMotor(int speedL, int speedR, bool dirL, bool dirR) {
  digitalWrite(DIR_L, dirL ? HIGH : LOW);
  digitalWrite(DIR_R, dirR ? HIGH : LOW);
  analogWrite(PWM_L, speedL);
  analogWrite(PWM_R, speedR);
}

void readEncoderRight() {
  bool A = digitalRead(encoderPinA_R);
  bool B = digitalRead(encoderPinB_R);
  encoderCount_R += (A == B) ? -1 : 1;
}

void readEncoderLeft() {
  bool A = digitalRead(encoderPinA_L);
  bool B = digitalRead(encoderPinB_L);
  encoderCount_L += (A == B) ? 1 : -1;
}
