/*
    nav.h

    Fuses raw sensor data into stable navigation values
    Supports: NXPSensorFusion, Madgwick, Mahony algorithms

    @authors Orfeas Magoulas
*/
#ifndef NAV_H
#define NAV_H

#include "sensors.h"
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>

/*
    Adafruit_NXPSensorFusion - Slowest, best accuracy
    Adafruit_Madgwick - Fair speed, fair accuracy
    Adafruit_Mahony - Fastest speed, lowest accuracy
*/
Adafruit_NXPSensorFusion filter;
// Adafruit_Mahony filter;
// Adafruit_Madgwick filter;

#define FILTER_UPDATE_RATE_HZ 100
#define IMU_UPDATE_RATE_HZ 100
#define GNSS_UPDATE_RATE_HZ 1
#define HEIGHT_UPDATE_RATE_HZ 10  // Update rate for the ultrasonic sensor

// #define AHRS_DEBUG_OUTPUT

class Nav
{
private:
    Sensors *sensors;
    int filterTimestamp;
    int imuTimestamp;
    int gnssTimestamp;
    int heightTimestamp;

    void updateFilter();
    void updateIMU();
    void updateGNSS();
    void updateHeight();

    // Ultrasonic sensor variables
    static const int NUM_HEIGHT_READINGS = 1;
    float heightReadings[NUM_HEIGHT_READINGS];
    int heightIndex;

public:
    // IMU
    float roll, pitch, heading;
    float qw, qx, qy, qz;
    float ax, ay, az;

    // GNSS
    float lat, lon, alt;
    int gnss_fix_type;
    int gnss_satellites;

    // Height measurement
    float height;  // Height in meters

    Nav(Sensors *sensors) : sensors(sensors) {}
    void init();
    void run();
};

/*
    Start filter at specified update rate
    Wait until pitch or roll error is less than 2 degrees for 5 seconds
*/
inline void Nav::init()
{
    Serial.println(F("[NAV] Initializing..."));
    filter.begin(FILTER_UPDATE_RATE_HZ);
    Serial.println(F("[NAV] Waiting for convergence..."));

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
    height = 0.0;

    unsigned long stableStartTime = 0;
    bool stable = true;

    while (!stable)
    {
        updateFilter();
        updateIMU();
        if (abs(pitch) < 2.0 && abs(roll) < 2.0)
        {
            if (stableStartTime == 0)
            {
                stableStartTime = millis();
            }
            else if (millis() - stableStartTime >= 5000)
            {
                stable = true;
            }
        }
        else
        {
            stableStartTime = 0;
        }

        delay(100);
    }
    printf("Pitch %lf, Roll: %lf\n", pitch, roll);
    Serial.println(F("[NAV] Convergence complete!"));
    Serial.println(F("[NAV] Initialization complete!"));
}

/*
    Update filter with latest sensor events
*/
inline void Nav::updateFilter()
{
    if ((millis() - filterTimestamp) < (1000 / FILTER_UPDATE_RATE_HZ))
    {
        return;
    }
    filterTimestamp = millis();

    // Get latest sensor events
    sensors_event_t accel, gyro, mag;
    sensors->accelerometer->getEvent(&accel);
    sensors->gyroscope->getEvent(&gyro);
    sensors->magnetometer->getEvent(&mag);

#if defined(AHRS_DEBUG_OUTPUT)
    Serial.print("I2C took ");
    Serial.print(millis() - timestamp);
    Serial.println(" ms");
#endif

    // Apply known calibrations to sensor events
    sensors->calibration.calibrate(mag);
    sensors->calibration.calibrate(accel);
    sensors->calibration.calibrate(gyro);

    // Convert gyro from Rad/s to Degree/s
    float gx, gy, gz;
    gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
    gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
    gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

    // Update filter with calibrated events
    filter.update(gx, gy, gz, accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

#if defined(AHRS_DEBUG_OUTPUT)
    Serial.print("Update took ");
    Serial.print(millis() - timestamp);
    Serial.println(" ms");
#endif
}

/*
    Update navigation IMU data
*/
inline void Nav::updateIMU()
{
    if ((millis() - imuTimestamp) < (1000 / IMU_UPDATE_RATE_HZ))
    {
        return;
    }
    imuTimestamp = millis();

    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    filter.getQuaternion(&qw, &qx, &qy, &qz);
    // filter.getLinearAcceleration(&ax, &ay, &az);
}

/*
    Update navigation GNSS data
*/
inline void Nav::updateGNSS()
{
    if ((millis() - gnssTimestamp) < (1000 / GNSS_UPDATE_RATE_HZ))
    {
        return;
    }
    gnssTimestamp = millis();

    if (!sensors->gnss.getPVT(1))
    {
        return;
    }

    gnss_satellites = sensors->gnss.getSIV();
    gnss_fix_type = sensors->gnss.getFixType();

    lat = sensors->gnss.getLatitude() / 1e7;
    lon = sensors->gnss.getLongitude() / 1e7;
    alt = sensors->gnss.getAltitudeMSL() / 1000.0;
}

/*
    Update height measurement from ultrasonic sensor
*/
inline void Nav::updateHeight()
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
        // No echo received; skip this reading
        return;
    }

    // Calculate distance in meters (Speed of sound = 343 m/s)
    float distance = (duration * 0.000343) / 2.0; // Convert microseconds to seconds and calculate distance

    // Store the reading in the buffer
    heightReadings[heightIndex] = distance;

    // Update index
    heightIndex = (heightIndex + 1) % NUM_HEIGHT_READINGS;

    // Compute the average
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
        height = sum / validReadings;
    }
}

/*
    Main run loop to update all sensor data
*/
inline void Nav::run()
{
    updateFilter();
    updateIMU();
    updateGNSS();
    updateHeight();
}

#endif