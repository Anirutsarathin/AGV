
#define PWM_EXTRA 8
#define DIR_EXTRA 9
int extraSpeed = 250;     

void setup() {
    Serial.begin(115200);
    pinMode(PWM_EXTRA, OUTPUT);
    pinMode(DIR_EXTRA, OUTPUT);
    analogWrite(PWM_EXTRA, 0);
}
void loop() {
  // put your main code here, to run repeatedly:
    Serial.println("⚙️ Extra Motor: เดินหน้า 3 วิ...");
    digitalWrite(DIR_EXTRA, HIGH);   // ขึ้น
    analogWrite(PWM_EXTRA, extraSpeed);
    // delay(100);
    // Serial.println("🔁 Extra Motor: ถอยหลัง 3 วิ...");
    // digitalWrite(DIR_EXTRA, LOW);    // ลง
    // analogWrite(PWM_EXTRA, extraSpeed);
    // delay(10000);
}
