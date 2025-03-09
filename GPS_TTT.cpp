 #include <TinyGPS++.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Adafruit_BMP085.h>
#include <HMC5883L_Simple.h>
MPU6050 accelgyro;
Adafruit_BMP085 bmp;
HMC5883L_Simple Compass;
double headingOffset;
#define MOTOR_PWM 5 
#define STEERING_SERVO 9
double c_lat,c_lon,targetLat,targetLon;
double distance,targetBearing ,heading,turnAngle ,heading_b,heading_t;
TinyGPSPlus gps;
Servo steeringServo;
int servoAngle;
const int num = 7;
double waypoints[8][2] = {
    {13.823348, 100.514480},
    {13.823386, 100.514450},
    {13.823424, 100.514419},
    {13.823473, 100.514373},
    {13.823537, 100.514328},
    {13.823578, 100.514289},
    {13.823649, 100.514328},
    {13.823715, 100.514349}
};
// 13.823578, 100.514289

double kalmanLat = 0.0, kalmanLon = 0.0;
double errEstLat = 1, errEstLon = 1;
double processNoise = 0.01;  // Process noise (adjustable)
double measurementNoise = 1; // Measurement noise (adjustable)

double kalmanFilter(double newValue, double &prevValue, double &errEst) {
    double kalmanGain = errEst / (errEst + measurementNoise);
    prevValue = prevValue + kalmanGain * (newValue - prevValue);
    errEst = (1 - kalmanGain) * errEst + processNoise;
    return prevValue;
}


int currentWaypoint = 0; 
#define FILTER_SIZE 10
float headingFilter[FILTER_SIZE];
int filterIndex = 0;

float getFilteredHeading() {
    headingFilter[filterIndex] = Compass.GetHeadingDegrees();
    filterIndex = (filterIndex + 1) % FILTER_SIZE;

    float sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
        sum += headingFilter[i];
    }
    return sum / FILTER_SIZE;
}
double distanceTo(double lat1, double lon1, double lat2, double lon2) {
    double R = 6371000; 
    double dLat = radians(lat2 - lat1);
    double dLon = radians(lon2 - lon1);
    double a = sin(dLat/2) * sin(dLat/2) +
               cos(radians(lat1)) * cos(radians(lat2)) *
               sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return R * c;
}

double bearingTo(double lat1, double lon1, double lat2, double lon2) {
    double dLon = radians(lon2 - lon1);
    double y = sin(dLon) * cos(radians(lat2));
    double x = cos(radians(lat1)) * sin(radians(lat2)) - sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);
    double bearing = degrees(atan2(y, x));
    
    return fmod((bearing + 360), 360); 
}
double calculateTurnAngle(double heading, double targetBearing) {
    double turn = targetBearing - heading;
    if (turn > 180) turn -= 360;
    if (turn < -180) turn += 360;
    return turn;
}
void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);
  steeringServo.attach(STEERING_SERVO);
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(11,INPUT);
  Wire.begin();
  Serial1.print("$PMTK220,100*2F\r\n");  delay(100);
  //Serial1.print("$PMTK397,0.0,50 .0,5.0,1.0,100.0*25\r\n");
  //Serial1.print("$PMTK251,115200*1F\r\n");delay(100);
 // Serial1.print("$PMTK397,0.0,50.0,5.0,1.0,100.0*25\r\n");
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }
  steeringServo.write(112);
  accelgyro.initialize();
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  accelgyro.setI2CBypassEnabled(true); 
  Compass.SetDeclination(0, 6, 'W');
  Compass.SetSamplingMode(COMPASS_SINGLE);
  Compass.SetScale(COMPASS_SCALE_130);
  Compass.SetOrientation(COMPASS_HORIZONTAL_X_NORTH);
  headingOffset = Compass.GetHeadingDegrees();
  delay(2000);
}
double getAdjustedHeading(double rawHeading) {
    double adjustedHeading = rawHeading - headingOffset;  
    if (adjustedHeading < 0) adjustedHeading += 360;
    if (adjustedHeading >= 360) adjustedHeading -= 360;

    return adjustedHeading;

    // adjust angle();
}
void loop() {
     while (Serial1.available() > 0) {
        gps.encode(Serial1.read());
        if (gps.location.isUpdated()) {
            c_lat = gps.location.lat();
            c_lon = gps.location.lng();
            kalmanLat = kalmanFilter(c_lat, kalmanLat, errEstLat);
            kalmanLon = kalmanFilter(c_lon, kalmanLon, errEstLon);
            targetLat = waypoints[currentWaypoint][0];
            targetLon = waypoints[currentWaypoint][1];
            distance = distanceTo(c_lat, c_lat, targetLat, targetLon);
            targetBearing = bearingTo(c_lat, c_lat, targetLat, targetLon);
//            heading_b = getFilteredHeading();
//            heading = getAdjustedHeading(heading_b);
//            heading_t = heading_b+20;
            heading = Compass.GetHeadingDegrees();
            turnAngle = calculateTurnAngle(heading, targetBearing);
            int sw = digitalRead(11);
            if(sw == LOW){
              currentWaypoint = 0;
              }
            if(turnAngle >= 90 && turnAngle <= 180){
              steeringServo.write(118);
              }
            else if(turnAngle <= -90&&turnAngle >= -180){
              steeringServo.write(108);
              }
              else {
               steeringServo.write(112); 
                
                }
            //turnAngle = fmod((targetBearing - heading + 540), 360) - 180;
            //servoAngle = map(turnAngle, -180, 180, 108, 118); 
            //steeringServo.write(servoAngle);
            if (distance < 5.0 && gps.hdop.hdop() < 2.0) {
                currentWaypoint++;
                if(currentWaypoint >= num){
                    currentWaypoint = num;
                  }
            }
            Serial.print(" \t");
            Serial.print(c_lat, 6);
            Serial.print(" \t");
            Serial.print(c_lon, 6);
            Serial.print("\t"); 
            Serial.print(kalmanLat, 6);
            Serial.print(" \t "); 
            Serial.print(kalmanLon, 6);
            Serial.print(" \t");
            Serial.print("Dt: "); 
            Serial.print(distance);
            Serial.print(" N.: "); 
            Serial.print(currentWaypoint);
            Serial.print(" h: "); 
            Serial.print(heading);
            Serial.print(" t: "); 
            Serial.print(targetBearing);
            Serial.print(" tA: "); 
            Serial.print(turnAngle);
            Serial.print(" SA: "); 
            Serial.println(gps.hdop.hdop());
        }
    }

}
