#include <TinyGPS++.h>
#include <Wire.h>
#include <HardwareSerial.h>

#define RXPin 16
#define TXPin 17
#define GPSBaud 9600
double c_lat, c_lon, dis, targetLat, targetLon, h;
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

// Encoder Pins
#define EncoderPinA_RIGHT 35
#define EncoderPinB_RIGHT 34
#define EncoderPinA_LEFT 33
#define EncoderPinB_LEFT 32

// Motor Control Pins
#define ENA 14  
#define IN1 12  
#define IN2 13
#define ENB 25   
#define IN3 26   
#define IN4 27

#define PPR 17334  
#define WHEEL_RADIUS 60.0  
#define TIME_INTERVAL 1000  
#define TARGET_RPM 10  

// Push Button
#define BUTTON_1 15
#define BUTTON_2 4
#define BUTTON_3 5

volatile long rawEncoderValue_RIGHT = 0;
volatile long rawEncoderValue_LEFT = 0;
volatile unsigned long prevTime = 0;

float distance_RIGHT = 0;
float distance_LEFT = 0;

bool moveMotor = false;
float targetDistance = 0;
float currentRPM = 0;
bool lastButtonState3 = HIGH;  

double waypoints[1][2] = {
    {13.820750, 100.515516},  
};

// ตัวแปรสำหรับกรองค่า GPS
const int filterSize = 5;  // เก็บค่าพิกัด 5 ค่า
double latBuffer[filterSize] = {0};
double lonBuffer[filterSize] = {0};
int filterIndex = 0;

// ฟังก์ชันเคลียร์ค่าเดิมก่อนบันทึกค่าใหม่
void resetWaypoints() {
    waypoints[0][0] = 0;
    waypoints[0][1] = 0;
}

// ฟังก์ชัน Moving Average Filter สำหรับ GPS
void updateGPSFilter(double newLat, double newLon) {
    latBuffer[filterIndex] = newLat;
    lonBuffer[filterIndex] = newLon;
    filterIndex = (filterIndex + 1) % filterSize;  // วนค่าไปเก็บในบัฟเฟอร์

    double sumLat = 0, sumLon = 0;
    for (int i = 0; i < filterSize; i++) {
        sumLat += latBuffer[i];
        sumLon += lonBuffer[i];
    }
    c_lat = sumLat / filterSize;
    c_lon = sumLon / filterSize;
}

// ฟังก์ชันบันทึกพิกัดปัจจุบันเมื่อกดปุ่ม
void saveCurrentLocation() {
    resetWaypoints();  // เคลียร์ค่าพิกัดก่อนบันทึกใหม่
    waypoints[0][0] = c_lat;
    waypoints[0][1] = c_lon;
    Serial.println("📌 Saved new waypoint!");
    Serial.print("Latitude: "); Serial.println(waypoints[0][0], 6);
    Serial.print("Longitude: "); Serial.println(waypoints[0][1], 6);
}

void IRAM_ATTR updateEncoder_RIGHT() {
    if (digitalRead(EncoderPinA_RIGHT) == digitalRead(EncoderPinB_RIGHT)) {
        rawEncoderValue_RIGHT++;
    } else {
        rawEncoderValue_RIGHT--;
    }
}

void IRAM_ATTR updateEncoder_LEFT() {
    if (digitalRead(EncoderPinA_LEFT) == digitalRead(EncoderPinB_LEFT)) {
        rawEncoderValue_LEFT++;
    } else {
        rawEncoderValue_LEFT--;
    }
}

double distanceTo(double lat1, double lon1, double lat2, double lon2) {
    double R = 6371000;  
    double dLat = radians(lat2 - lat1);
    double dLon = radians(lon2 - lon1);
    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(radians(lat1)) * cos(radians(lat2)) *
               sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R * c;  
}

void setup() {
    Serial.begin(115200);
    gpsSerial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);

    Serial.println("Initializing GPS...");

    pinMode(EncoderPinA_RIGHT, INPUT_PULLUP);
    pinMode(EncoderPinB_RIGHT, INPUT_PULLUP);
    pinMode(EncoderPinA_LEFT, INPUT_PULLUP);
    pinMode(EncoderPinB_LEFT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(EncoderPinA_RIGHT), updateEncoder_RIGHT, RISING);
    attachInterrupt(digitalPinToInterrupt(EncoderPinA_LEFT), updateEncoder_LEFT, RISING);

    pinMode(BUTTON_1, INPUT_PULLUP);
    pinMode(BUTTON_2, INPUT_PULLUP);
    pinMode(BUTTON_3, INPUT_PULLUP);

    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
}

void loop() {
    read_dit();

    // อ่านค่าพิกัด GPS และกรองค่า
    while (gpsSerial.available()) {
        gps.encode(gpsSerial.read());
        if (gps.location.isUpdated()) {
            updateGPSFilter(gps.location.lat(), gps.location.lng());
            h = gps.hdop.hdop();
        }
    }

    unsigned long currentTime = millis();
    if (currentTime - prevTime >= 50) {
        bool currentState3 = digitalRead(BUTTON_3);

        // ตรวจจับว่าปุ่มถูกกดลง (HIGH → LOW)
        if (lastButtonState3 == HIGH && currentState3 == LOW) {
            Serial.println("🎯 Button 3 Pressed -> Saving current location");
            saveCurrentLocation();
        }
        lastButtonState3 = currentState3;  

        targetLat = waypoints[0][0];
        targetLon = waypoints[0][1];
        dis = distanceTo(c_lat, c_lon, targetLat, targetLon);

        Serial.print("\t");
        Serial.print(c_lat, 6);
        Serial.print("\t");
        Serial.print(c_lon, 6);
        Serial.print(": "); Serial.print(dis);
        Serial.print(": "); Serial.print(h);
        Serial.print(" | Distance RIGHT (mm): "); Serial.print(distance_RIGHT);
        Serial.print(" | Distance LEFT (mm): "); Serial.println(distance_LEFT);

        if (dis <= 10) {
            stopMotor();
            moveMotor = false;
        } else {
            driveMotor(120);
        }
    }
}

void driveMotor(int speed1) {
    analogWrite(ENA, 150);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    analogWrite(ENB, 150);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void stopMotor() {
    analogWrite(ENA, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);

    analogWrite(ENB, 0);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

void read_dit() {
    unsigned long currentTime = millis();
    if (currentTime - prevTime >= TIME_INTERVAL) {
        int deltaEncoder_RIGHT = rawEncoderValue_RIGHT;
        int deltaEncoder_LEFT = rawEncoderValue_LEFT;

        rawEncoderValue_RIGHT = 0;
        rawEncoderValue_LEFT = 0;

        float RPM_RIGHT = (deltaEncoder_RIGHT * 60.0) / PPR;
        float RPM_LEFT = (deltaEncoder_LEFT * 60.0) / PPR;

        float linearVelocity_RIGHT = (2 * PI * WHEEL_RADIUS * RPM_RIGHT) / 60.0;
        float linearVelocity_LEFT = (2 * PI * WHEEL_RADIUS * RPM_LEFT) / 60.0;

        distance_RIGHT += linearVelocity_RIGHT * (TIME_INTERVAL / 1000.0);
        distance_LEFT += linearVelocity_LEFT * (TIME_INTERVAL / 1000.0);
        prevTime = currentTime;
    }
}
