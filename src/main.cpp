#include <Arduino.h>

#define ENCODEROUTPUT 663  // จำนวนพัลส์ต่อรอบของมอเตอร์
#define HALLSEN_A 34  // ขา A ของ Hall Sensor
#define MOTOR1A 18    // ขาควบคุมทิศทางมอเตอร์
#define MOTOR1B 19    // ขาควบคุมทิศทางมอเตอร์
#define PWM_MOTOR 5   // ขา PWM ควบคุมความเร็วของมอเตอร์

volatile long encoderValue = 0;  // ตัวแปรเก็บค่าพัลส์จาก Encoder
int interval = 1000;             // จับเวลาทุก 1 วินาที
long previousMillis = 0;         // เก็บค่ามิลลิวินาทีล่าสุด
int rpm = 0;   // ตัวแปรเก็บค่าคำนวณ RPM
int motorPwm = 200;  // ค่า PWM เริ่มต้น (0-255)

void IRAM_ATTR updateEncoder() {
    encoderValue++;  // เพิ่มค่าพัลส์ทุกครั้งที่มีสัญญาณ RISING
}

void setup() {
    Serial.begin(115200);
    
    pinMode(MOTOR1A, OUTPUT);
    pinMode(MOTOR1B, OUTPUT);
    pinMode(PWM_MOTOR, OUTPUT);
    pinMode(HALLSEN_A, INPUT);

    // ตั้งค่ามอเตอร์ให้หมุนไปข้างหน้า
    digitalWrite(MOTOR1A, HIGH);
    digitalWrite(MOTOR1B, LOW);
    
    // ตั้งค่า PWM บน ESP32
    ledcSetup(0, 1000, 8); // Channel 0, 1 kHz, 8-bit resolution
    ledcAttachPin(PWM_MOTOR, 0);
    ledcWrite(0, motorPwm); // ตั้งค่าความเร็วเริ่มต้น

    // ตั้งค่า Interrupt ขา HALLSEN_A
    attachInterrupt(digitalPinToInterrupt(HALLSEN_A), updateEncoder, RISING);
}

void loop() {
    long currentMillis = millis();
    
    // คำนวณค่า RPM ทุก 1 วินาที
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        rpm = (float)(encoderValue * 60 / ENCODEROUTPUT);

        Serial.print("Pulses per second: ");
        Serial.print(encoderValue);
        Serial.print(" | Expected at 500 RPM: ~5525 pulses");
        Serial.print(" | Calculated RPM: ");
        Serial.print(rpm);
        Serial.println(" RPM");

        encoderValue = 0;  // รีเซ็ตค่าพัลส์หลังจากคำนวณ
    }
}
