#include <TinyGPS++.h>
#include <Wire.h>
#include <HardwareSerial.h>

#define RXPin 16
#define TXPin 17
#define GPSBaud 9600

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

// 📌 เปิด GNSS ครบชุด (GPS, GLONASS, Galileo, BeiDou, QZSS)
const uint8_t enableGNSS[] = {
  0xB5, 0x62, 0x06, 0x3E, 0x2C, 0x00, 
  0x00, 0x00, 0x20, 0x07, 0x07, 0x00, 0x00, 0x00, 0x00, 0x01,
  0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,  
  0x01, 0x01, 0x01, 0x01, 
  0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 
  0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x01,
  0xD8, 0x5B
};

// 📌 เพิ่มอัตราการอัปเดตเป็น 5Hz
const uint8_t setRate5Hz[] = {
  0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
  0xFA, 0x00, 0x01, 0x00, 0x00, 0x00,
  0xC0, 0x7E
};

// 📌 ฟังก์ชันส่งคำสั่ง UBX
void sendUBXCommand(const uint8_t *command, size_t length) {
  for (size_t i = 0; i < length; i++) {
    gpsSerial.write(command[i]);
  }
}

void setup() {
    Serial.begin(115200);
    gpsSerial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);

    Serial.println("Initializing GPS...");
    sendUBXCommand(enableGNSS, sizeof(enableGNSS));
    delay(500);
    sendUBXCommand(setRate5Hz, sizeof(setRate5Hz));
    delay(500);

    Serial.println("GNSS Enabled with 5Hz update rate.");
}

void loop() {
    while (gpsSerial.available()) {
        gps.encode(gpsSerial.read());
    }

    if (gps.location.isUpdated()) {
        Serial.printf("Lat:%.8f, Lon:%.8f | Spd:%.2fm/s | ", 
                      gps.location.lat(), gps.location.lng(), gps.speed.mps());

        // ✅ ปรับปรุงการอ่านค่า HDOP
        float hdop = gps.hdop.hdop();
        if (hdop == 0.0) {
            Serial.print("HDOP:N/A");
        } else {
            Serial.printf("HDOP:%.2f", hdop);
        }

        Serial.printf(" | Sat:%d | ", gps.satellites.value());

        // ✅ แสดงจำนวนดาวเทียมที่มองเห็น + ใช้งานจริง
        showGNSSInfo();
        showUsedSatellites();
        Serial.println();
    }
}

// 📌 ฟังก์ชันแสดงจำนวนดาวเทียมที่มองเห็นจากแต่ละระบบ
void showGNSSInfo() {
    int gpsCount = 0, glonassCount = 0, galileoCount = 0, beidouCount = 0, qzssCount = 0;

    gpsSerial.println("$GNGSV");
    delay(200);
    String gsvData = readSerialData();

    gpsCount = countOccurrences(gsvData, "GP");
    glonassCount = countOccurrences(gsvData, "GL");
    galileoCount = countOccurrences(gsvData, "GA");
    beidouCount = countOccurrences(gsvData, "BD");
    qzssCount = countOccurrences(gsvData, "QZ");

    Serial.printf("GPS:%d | GLO:%d | GAL:%d | BDS:%d | QZS:%d | ", 
                  gpsCount, glonassCount, galileoCount, beidouCount, qzssCount);
}

// 📌 ฟังก์ชันแสดงจำนวนดาวเทียมที่ **"ถูกใช้งานจริง"** จาก GNGSA
void showUsedSatellites() {
    gpsSerial.println("$GNGSA");
    delay(200);
    String gsaData = readSerialData();

    int usedSatellites = countUsedSatellites(gsaData);
    
    Serial.printf("Used:%d", usedSatellites);
}

// 📡 อ่านข้อมูลจาก GPS
String readSerialData() {
    String data = "";
    unsigned long start = millis();
    while (millis() - start < 500) {
        while (gpsSerial.available()) {
            char c = gpsSerial.read();
            data += c;
        }
    }
    return data;
}

// 🔍 นับจำนวนคำที่พบใน String
int countOccurrences(String data, String keyword) {
    int count = 0;
    int index = 0;
    
    while ((index = data.indexOf(keyword, index)) != -1) {
        count++;
        index += keyword.length();
    }
    return count;
}

// ✅ ฟังก์ชันหาจำนวนดาวเทียมที่ **ใช้งานจริง** จาก GNGSA
int countUsedSatellites(String gsaData) {
    int count = 0;
    int index = 0;
    
    while ((index = gsaData.indexOf(",", index)) != -1) {
        count++;
        index++;
    }

    // ✅ ค่าใน GNGSA จะมี ",,," หลายตัว ต้องลบออก 3 ตำแหน่งแรก
    if (count > 3) count -= 3;
    
    return count;
}
