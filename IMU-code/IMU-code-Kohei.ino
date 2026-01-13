#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)
#define FRAMES_PER_LINE 7   // ✅ 7フレームで改行

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// Velocity storage
float vx = 0, vy = 0, vz = 0;
unsigned long lastTime = 0;

// ✅ フレームカウンタ
int frameCount = 0;

// ------------------------------
// millis() → 00:00:00.000 形式
// ------------------------------
void printFormattedTime(unsigned long ms)
{
 unsigned long totalSeconds = ms / 1000;
 unsigned long hours   = (totalSeconds / 3600) % 24;
 unsigned long minutes = (totalSeconds / 60) % 60;
 unsigned long seconds = totalSeconds % 60;
 unsigned long millisPart = ms % 1000;

 if (hours < 10) Serial.print("0");
 Serial.print(hours); Serial.print(":");

 if (minutes < 10) Serial.print("0");
 Serial.print(minutes); Serial.print(":");

 if (seconds < 10) Serial.print("0");
 Serial.print(seconds); Serial.print(".");

 if (millisPart < 100) Serial.print("0");
 if (millisPart < 10)  Serial.print("0");
 Serial.print(millisPart);
}

// ------------------------------
// Setup
// ------------------------------
void setup(void)
{
 Serial.begin(115200);
 while (!Serial) delay(10);

 Serial.println("BNO055 7-Frame CSV Logger");

 if (!bno.begin())
 {
   Serial.println("No BNO055 detected. Check wiring!");
   while (1);
 }

 delay(1000);
 bno.setExtCrystalUse(true);
 lastTime = millis();
}

// ------------------------------
// MAIN LOOP
// ------------------------------
void loop(void)
{
 sensors_event_t orientationData, accelData, linearAccelData, gravityData;

 bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
 bno.getEvent(&accelData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
 bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
 bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

 // Δt 計算
 unsigned long currentTime = millis();
 float dt = (currentTime - lastTime) / 1000.0;
 lastTime = currentTime;

 // 速度積分
 vx += linearAccelData.acceleration.x * dt;
 vy += linearAccelData.acceleration.y * dt;
 vz += linearAccelData.acceleration.z * dt;

 float speed = sqrt(vx * vx + vy * vy + vz * vz);

 // キャリブレーション取得
 uint8_t system, gyro, accel, mag;
 bno.getCalibration(&system, &gyro, &accel, &mag);

 // ------------------------------
 // ✅ 1フレーム分のCSVを出力（まだ改行しない）
 // ------------------------------
 printFormattedTime(currentTime); Serial.print(",");

 Serial.print(orientationData.orientation.x); Serial.print(",");
 Serial.print(orientationData.orientation.y); Serial.print(",");
 Serial.print(orientationData.orientation.z); Serial.print(",");

 Serial.print(linearAccelData.acceleration.x); Serial.print(",");
 Serial.print(linearAccelData.acceleration.y); Serial.print(",");
 Serial.print(linearAccelData.acceleration.z); Serial.print(",");

 Serial.print(speed); Serial.print(",");

 Serial.print(system); Serial.print(",");
 Serial.print(gyro); Serial.print(",");
 Serial.print(accel); Serial.print(",");
 Serial.print(mag);

 // ✅ フレーム区切りのカンマ
 Serial.print(",");

 frameCount++;

 // ------------------------------
 // ✅ 7フレームごとに改行
 // ------------------------------
 if (frameCount >= FRAMES_PER_LINE)
 {
   Serial.println();   // ← ここでだけ改行
   frameCount = 0;
 }

 delay(BNO055_SAMPLERATE_DELAY_MS);
}


