/*
    nav.h
    
    Fuses raw sensor data into stable navigation values
    Supports: NXPSensorFusion, Madgwick, Mahony algorithms
    
    @authors Orfeas Magoulas
*/
#ifndef NAV_H
#define NAV_H

#include "Sensors.h"
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

// #define AHRS_DEBUG_OUTPUT

class Nav
{
private:
    Sensors* sensors;
    int filterTimestamp;
    int imuTimestamp;
    int gnssTimestamp;
    void updateFilter();
    void updateIMU();
    void updateGNSS();
public:
    // IMU
    float roll, pitch, heading;
    float qw, qx, qy, qz;
    float ax, ay, az;

    // GNSS
    float lat, lon, alt;
    int gnss_fix_type;
    int gnss_satellites;

    Nav(Sensors* sensors) : sensors(sensors) {}
    void init();
    void run();
};

/*
    Start filter at specified update rate
*/
inline void Nav::init()
{
    Serial.println(F("[NAV] Initializing..."));
    filter.begin(FILTER_UPDATE_RATE_HZ);
    Serial.println(F("[NAV] Initialization complete!"));
}

/*
    Update filter with latest sensor events
*/
inline void Nav::updateFilter()
{
    if ((millis() - filterTimestamp) < (1000 / FILTER_UPDATE_RATE_HZ)) {
        return;
    }
    filterTimestamp = millis();
    
    // Get latest sensor events
    sensors_event_t accel, gyro, mag;
    sensors->accelerometer->getEvent(&accel);
    sensors->gyroscope->getEvent(&gyro);
    sensors->magnetometer->getEvent(&mag);

    #if defined(AHRS_DEBUG_OUTPUT)
    Serial.print("I2C took "); Serial.print(millis() - timestamp); Serial.println(" ms");
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
    Serial.print("Update took "); Serial.print(millis() - timestamp); Serial.println(" ms");
    #endif
}

/*
    Update navigation IMU data
*/
inline void Nav::updateIMU()
{
    if ((millis() - imuTimestamp) < (1000 / IMU_UPDATE_RATE_HZ)) {
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
    if ((millis() - gnssTimestamp) < (1000 / GNSS_UPDATE_RATE_HZ)) {
        return;
    }
    gnssTimestamp = millis();

    if (!sensors->gnss.getPVT(1)) {
        return;
    }

    gnss_satellites = sensors->gnss.getSIV();
    gnss_fix_type = sensors->gnss.getFixType();

    lat = sensors->gnss.getLatitude() / 1e7;
    lon = sensors->gnss.getLongitude() / 1e7;
    alt = sensors->gnss.getAltitudeMSL() / 1000.0;
}

inline void Nav::run()
{
    updateFilter();
    updateIMU();
    updateGNSS();
}

#endif
