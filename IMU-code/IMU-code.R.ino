#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)

// Create sensor object
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// Velocity storage
float vx = 0, vy = 0, vz = 0;
unsigned long lastTime = 0;


void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.println(sensor.max_value);
  Serial.print  ("Min Value:    "); Serial.println(sensor.min_value);
  Serial.print  ("Resolution:   "); Serial.println(sensor.resolution);
  Serial.println("------------------------------------\n");
  delay(500);
}


void displayCalStatus(void)
{
  uint8_t system, gyro, accel, mag;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  Serial.print("Calib  Sys:");
  Serial.print(system);
  Serial.print(" G:");
  Serial.print(gyro);
  Serial.print(" A:");
  Serial.print(accel);
  Serial.print(" M:");
  Serial.print(mag);
  Serial.println();
}

void setup(void)
{
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("BNO055 Orientation + Acceleration + Speed Demo\n");

  if (!bno.begin())
  {
    Serial.println("No BNO055 detected. Check wiring or I2C address!");
    while (1);
  }

  delay(1000);

  displaySensorDetails();
  bno.setExtCrystalUse(true);
  lastTime = millis();
}


// MAIN LOOP
void loop(void)
{
  sensors_event_t orientationData, accelData, linearAccelData, gravityData;


  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&accelData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
 
  Serial.print("Orientation (deg)  X:");
  Serial.print(orientationData.orientation.x);
  Serial.print(" Y:");
  Serial.print(orientationData.orientation.y);
  Serial.print(" Z:");
  Serial.println(orientationData.orientation.z);


  // Print linear acceleration 

  Serial.print("Linear Accel (m/s^2)  X:");
  Serial.print(linearAccelData.acceleration.x);
  Serial.print(" Y:");
  Serial.print(linearAccelData.acceleration.y);
  Serial.print(" Z:");
  Serial.println(linearAccelData.acceleration.z);


  // Speed calculation

  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;  // seconds
  lastTime = currentTime;

  //  acceleration - velocity
  vx += linearAccelData.acceleration.x * dt;
  vy += linearAccelData.acceleration.y * dt;
  vz += linearAccelData.acceleration.z * dt;

  // Speed magnitude
  float speed = sqrt(vx*vx + vy*vy + vz*vz);

  Serial.print("Speed (m/s): ");
  Serial.println(speed);


  displayCalStatus();

  Serial.println("--------------------------------------\n");
  delay(BNO055_SAMPLERATE_DELAY_MS);
}


