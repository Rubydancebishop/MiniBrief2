#include <Adafruit_BNO055.h>

const int SENSOR_1_PIN = A0;
const int SENSOR_2_PIN = A1;
const int BAUD_RATE = 9600;
const int DELAY_MS = 50; 

void setup()
{
    Serial.begin(BAUD_RATE);
    while (!Serial)
    {
        ;
    }

    Serial.println("Serial Bridge - Two Sensors");
    Serial.println("Ready to send data!");
}

void loop()
{
    // Read both analog sensors
    int sensor1Value = analogRead(SENSOR_1_PIN);
    int sensor2Value = analogRead(SENSOR_2_PIN);

 
    Serial.print(sensor1Value);
    Serial.print(",");
    Serial.println(sensor2Value);

   
    delay(DELAY_MS);
}