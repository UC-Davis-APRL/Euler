/*
    sensors.h
    
    Initializes & configures sensors
    Built for: Adafruit LIS3MDL, Adafruit LSM6DSOX, Sparkfun U-BLOX_GNSS
    
    @authors Orfeas Magoulas
*/
#ifndef SENSORS_H
#define SENSORS_H

#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DSOX.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <Adafruit_Sensor_Calibration.h>
#include <InternalTemperature.h>

#define PRINT_IMU_DETAILS

class Sensors
{
private:
    int timestamp;
    Adafruit_LIS3MDL lis3mdl;
    Adafruit_LSM6DSOX lsm6dsox;
public:
    Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;
    Adafruit_Sensor_Calibration_EEPROM calibration;
    SFE_UBLOX_GNSS gnss;
    Sensors(){}
    void init();
    void initIMU();
    void initGNSS();
};

/*
    Load calibration data
    Initialize IMU & GNSS
*/
inline void Sensors::init()
{
    if (!calibration.begin()) {
        Serial.println(F("Failed to initialize IMU calibration."));
    } else if (!calibration.loadCalibration()) {
        Serial.println(F("No IMU calibration data found in EEPROM."));
    }

    initIMU();
    initGNSS();
}

/*
    Locate LSM6DSOX, LIS3MDL
    Configure sensor ranges, data rates, operation modes
    Print sensor details
*/
inline void Sensors::initIMU()
{
    while (!lsm6dsox.begin_I2C()) {
        Serial.println(F("LSM6DSOX not detected at default I2C address. Retrying..."));
        delay (1000);
    }

    while (!lis3mdl.begin_I2C()) {
        Serial.println(F("LIS3MDL not detected at default I2C address. Retrying..."));
        delay (1000);
    }

    accelerometer = lsm6dsox.getAccelerometerSensor();
    gyroscope = lsm6dsox.getGyroSensor();
    magnetometer = &lis3mdl;

    lsm6dsox.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    lsm6dsox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

    lsm6dsox.setAccelDataRate(LSM6DS_RATE_104_HZ);
    lsm6dsox.setGyroDataRate(LSM6DS_RATE_104_HZ);
    lis3mdl.setDataRate(LIS3MDL_DATARATE_1000_HZ);
    lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
    lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);

    #if defined(PRINT_IMU_DETAILS)
    accelerometer->printSensorDetails();
    gyroscope->printSensorDetails();
    magnetometer->printSensorDetails();
    #endif
}

inline void Sensors::initGNSS()
{
    // gnss.enableDebugging();
    while (!gnss.begin()) {
        Serial.println(F("U-BLOX M10 GNSS not detected at default I2C address. Retrying..."));
        delay (1000);
    }
    gnss.setI2COutput(COM_TYPE_UBX);
    // gnss.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
}

#endif