/*
    sensors.h

    Initializes & configures sensors
    Built for: Adafruit LIS3MDL, Adafruit LSM6DSOX, Sparkfun U-BLOX_GNSS

    @authors Orfeas Magoulas
*/
#ifndef SENSORS_H
#define SENSORS_H

#include <SparkFun_u-blox_GNSS_v3.h>

// #define PRINT_IMU_DETAILS

#define NUM_HEIGHT_READINGS 1
#define HEIGHT_UPDATE_RATE_HZ 10


class Sensors
{
private:
    int timestamp;
    float heightReadings[NUM_HEIGHT_READINGS];
    int heightTimestamp;
    int heightIndex;
public:
    SFE_UBLOX_GNSS gnss;
    Sensors() {}
    void init();
    void initGNSS();
    void initUltrasonic();
    void updateUltrasonic();
    float heightRaw;
};

/*
    Initialize IMU & GNSS
*/
inline void Sensors::init()
{
    Serial.println(F("[SENSORS] Initializing..."));
    initUltrasonic();
    // initGNSS();
    Serial.println(F("[SENSORS] Initialization complete!"));
}

inline void Sensors::initUltrasonic()
{
    // Initialize ultrasonic sensor pins
    pinMode(33, OUTPUT); // Trigger pin
    pinMode(34, INPUT);  // Echo pin

    // Initialize height readings
    heightIndex = 0;
    heightTimestamp = 0;
    for (int i = 0; i < NUM_HEIGHT_READINGS; i++)
    {
        heightReadings[i] = 0.0;
    }
    heightRaw = 0.0;
}


inline void Sensors::initGNSS()
{
    int attempt = 0;
    // gnss.enableDebugging();
    while (!gnss.begin() && attempt < 1)
    {
        attempt++;
        Serial.println(F("[SENSORS] U-BLOX M10 GNSS not detected at default I2C address. Retrying..."));
        delay(3000);
    }

    if (!gnss.begin()) {
        Serial.println(F("[SENSORS] U-BLOX M10 GNSS not detected at default I2C address. Starting without GNSS!"));

    }

    gnss.setI2COutput(COM_TYPE_UBX);
    // gnss.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
}

/*
    Update height measurement from ultrasonic sensor
*/
inline void Sensors::updateUltrasonic()
{
    if ((millis() - heightTimestamp) < (1000 / HEIGHT_UPDATE_RATE_HZ))
    {
        return;
    }
    heightTimestamp = millis();

    // Send trigger pulse
    digitalWrite(33, LOW);
    delayMicroseconds(2);
    digitalWrite(33, HIGH);
    delayMicroseconds(10);
    digitalWrite(33, LOW);

    // Read echo pulse duration
    unsigned long duration = pulseIn(34, HIGH, 30000); // Timeout after 30ms

    if (duration == 0)
    {
        return;
    }

    // Calculate distance in meters (M = 343 m/s)
    float distance = (duration * 0.000343) / 2.0; // microseconds to seconds

    heightReadings[heightIndex] = distance;
    heightIndex = (heightIndex + 1) % NUM_HEIGHT_READINGS;

    // Avg the readings
    float sum = 0.0;
    int validReadings = 0;
    for (int i = 0; i < NUM_HEIGHT_READINGS; i++)
    {
        if (heightReadings[i] > 0)
        {
            sum += heightReadings[i];
            validReadings++;
        }
    }

    if (validReadings > 0)
    {
        heightRaw = sum / validReadings;
    }
}

#endif