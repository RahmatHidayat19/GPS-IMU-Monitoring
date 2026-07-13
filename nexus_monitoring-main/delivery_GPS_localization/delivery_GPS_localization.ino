#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Wire.h>

SoftwareSerial gpsSerial(2, 3);
TinyGPSPlus gps;

const int MPU = 0x68;

float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY;
float gyroAngleX, gyroAngleY;
float roll, pitch, yaw;

float elapsedTime, currentTime, previousTime;

void setup() {

  Serial.begin(115200);
  gpsSerial.begin(9600);  // GPS modules usually use 9600

  Wire.begin();

  // Wake up MPU6050
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  delay(1000);
}

void loop() {

  parseGPS();
  readIMU();

  sendData();

  delay(50);  // ~20 Hz
}

void parseGPS() {

  while (gpsSerial.available()) {
    if (gps.encode(gpsSerial.read())) {
      updateGPS();
    }
  }
}

void readIMU() {

  // ===== Accelerometer =====
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU, 6, true);

  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;

  accAngleX = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 180 / PI;
  accAngleY = atan(-AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 180 / PI;

  // ===== Time =====
  previousTime = currentTime;
  currentTime = millis();

  elapsedTime = (currentTime - previousTime) / 1000.0;

  // ===== Gyroscope =====
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU, 6, true);

  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

  // Integrate gyro data
  gyroAngleX += GyroX * elapsedTime;
  gyroAngleY += GyroY * elapsedTime;
  yaw += GyroZ * elapsedTime;

  // Complementary filter
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
}
void updateGPS() {
  Serial.print(gps.location.lat(), 6);
  Serial.print(",");

  Serial.print(gps.location.lng(), 6);
  Serial.print(",");
}
void sendData() {

  // GPS valid?
  if (gps.location.isValid()) {
    parseGPS();
  } else {

    Serial.print("GPS invalid");
  }

  Serial.print(roll);
  Serial.print(",");

  Serial.print(pitch);
  Serial.print(",");

  Serial.println(yaw);
}