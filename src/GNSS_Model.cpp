#include <Wire.h>
#include <TinyGPSPlus.h>
#include <PID_v1.h>
#include <Kalman.h>
#include <SD.h>
#include <SPI.h>
#include <HardwareSerial.h>

// ** พารามิเตอร์ GNSS และ IMU **
#define GPS_RX 16
#define GPS_TX 17
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;
Kalman kalmanLat, kalmanLon;

// ** มอเตอร์และ Encoder **
#define MOTOR_PWM 5
#define MOTOR_IN1 18
#define MOTOR_IN2 19
#define ENCODER_A 34
#define ENCODER_B 35
volatile int pulseCount = 0;

// ** PID และ Motion Control **
double setSpeed = 100;
double inputSpeed, outputSpeed;
double Kp = 2.5, Ki = 4, Kd = 1.5;
PID speedPID(&inputSpeed, &outputSpeed, &setSpeed, Kp, Ki, Kd, DIRECT);

// ** พิกัดเป้าหมาย **
double targetLat = 13.73077;
double targetLon = 100.77183;

// ** ค่าตำแหน่งปัจจุบัน **
double currentLat, currentLon, heading;
unsigned long lastTime = 0;

// ** MPC (Model Predictive Control) **
double predictedSpeed, predictedAngle;

// ** LQR (Linear Quadratic Regulator) **
double Q = 0.01, R = 0.1; // ค่า Cost Function

// ** Adaptive Kalman Filter (AKF) **
double adaptiveQ = 1, adaptiveR = 5;
void updateAdaptiveKalman() {
    adaptiveQ = adaptiveQ * 0.95 + (currentLat - targetLat) * 0.05;
    adaptiveR = adaptiveR * 0.95 + (currentLon - targetLon) * 0.05;
}

// ** คำนวณระยะทางด้วย Vincenty's Formula **
double haversine(double lat1, double lon1, double lat2, double lon2) {
    double R = 6371000;
    double dLat = radians(lat2 - lat1);
    double dLon = radians(lon2 - lon1);
    lat1 = radians(lat1);
    lat2 = radians(lat2);

    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    return R * c;
}

double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
    double dLon = radians(lon2 - lon1);
    lat1 = radians(lat1);
    lat2 = radians(lat2);

    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) -
               sin(lat1) * cos(lat2) * cos(dLon);
    
    return fmod(degrees(atan2(y, x)) + 360.0, 360.0);
}

// ** MPC: คำนวณล่วงหน้า **
void computeMPC(double targetBearing, double dt) {
    double error = targetBearing - heading;
    predictedAngle = heading + (error * dt);
    predictedSpeed = setSpeed * (1 - fabs(error) / 90);
}

// ** LQR: ปรับการควบคุมความสมดุลของเส้นทาง **
double computeLQR(double error) {
    return -Q * error + R * predictedSpeed;
}

// ** Dead Reckoning: คำนวณตำแหน่งเมื่อ GNSS ขาดหาย **
void updatePositionDeadReckoning(double dt) {
    double velocity = inputSpeed;
    currentLat += velocity * cos(heading) * dt / 111320.0;
    currentLon += velocity * sin(heading) * dt / (111320.0 * cos(currentLat));
}

// ** Dynamic Speed Adjustment **
void dynamicSpeedControl(double distance) {
    if (distance > 5) {
        setSpeed = 150;
    } else if (distance > 1) {
        setSpeed = 80;
    } else {
        setSpeed = 30;
    }
}

// ** อ่านค่าจาก GNSS **
void readGPS() {
    while (gpsSerial.available()) {
        gps.encode(gpsSerial.read());
    }

    if (gps.location.isValid()) {
        double rawLat = gps.location.lat();
        double rawLon = gps.location.lng();

        currentLat = kalmanLat.getAngle(rawLat, adaptiveQ);
        currentLon = kalmanLon.getAngle(rawLon, adaptiveR);
    }
}

// ** อ่านค่าความเร็วจาก Encoder **
void IRAM_ATTR countPulse() {
    pulseCount++;
}

// ** บันทึกข้อมูลลง SD Card **
void logGPSData() {
    File file = SD.open("/gps_log.csv", FILE_APPEND);
    if (file) {
        file.print(currentLat, 6);
        file.print(",");
        file.println(currentLon, 6);
        file.close();
    }
}

void setup() {
    Serial.begin(115200);
    gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

    pinMode(MOTOR_PWM, OUTPUT);
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    pinMode(ENCODER_A, INPUT);
    pinMode(ENCODER_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), countPulse, RISING);

    SD.begin(5); // เริ่มต้น SD Card
    speedPID.SetMode(AUTOMATIC);
}

void loop() {
    readGPS();
    double distanceToTarget = haversine(currentLat, currentLon, targetLat, targetLon);
    double targetBearing = calculateBearing(currentLat, currentLon, targetLat, targetLon);

    dynamicSpeedControl(distanceToTarget);
    computeMPC(targetBearing, 0.1);
    double lqrAdjust = computeLQR(targetBearing - heading);

    updateAdaptiveKalman();
    speedPID.Compute();
    analogWrite(MOTOR_PWM, outputSpeed);

    if (abs(targetBearing - heading) > 5) {
        digitalWrite(MOTOR_IN1, targetBearing > heading ? HIGH : LOW);
        digitalWrite(MOTOR_IN2, targetBearing > heading ? LOW : HIGH);
    } else {
        digitalWrite(MOTOR_IN1, HIGH);
        digitalWrite(MOTOR_IN2, HIGH);
    }

    logGPSData();
    Serial.print("Distance: ");
    Serial.print(distanceToTarget);
    Serial.println(" m");

    delay(100);
}
