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

// #define PRINT_IMU_DETAILS

#define NUM_HEIGHT_READINGS 1
#define HEIGHT_UPDATE_RATE_HZ 10


class Sensors
{
private:
    int timestamp;
    Adafruit_LIS3MDL lis3mdl;
    Adafruit_LSM6DSOX lsm6dsox;
    float heightReadings[NUM_HEIGHT_READINGS];
    int heightTimestamp;
    int heightIndex;
public:
    Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;
    Adafruit_Sensor_Calibration_EEPROM calibration;
    SFE_UBLOX_GNSS gnss;
    Sensors() {}
    void init();
    void initIMU();
    void initGNSS();
    void initUltrasonic();
    void updateUltrasonic();
    float heightRaw;
};

/*
    Load calibration data
    Initialize IMU & GNSS
*/
inline void Sensors::init()
{
    Serial.println(F("[SENSORS] Initializing..."));
    if (!calibration.begin())
    {
        Serial.println(F("[SENSORS] Failed to initialize IMU calibration."));
    }
    else if (!calibration.loadCalibration())
    {
        Serial.println(F("[SENSORS] No IMU calibration data found in EEPROM."));
    }

    initIMU();
    initGNSS();
    initUltrasonic();
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


/*
    Locate LSM6DSOX, LIS3MDL
    Configure sensor ranges, data rates, operation modes
    Print sensor details
*/
inline void Sensors::initIMU()
{
    while (!lsm6dsox.begin_I2C())
    {
        Serial.println(F("[SENSORS] LSM6DSOX not detected at default I2C address. Retrying..."));
        delay(3000);
    }

    while (!lis3mdl.begin_I2C())
    {
        Serial.println(F("[SENSORS] LIS3MDL not detected at default I2C address. Retrying..."));
        delay(3000);
    }

    accelerometer = lsm6dsox.getAccelerometerSensor();
    gyroscope = lsm6dsox.getGyroSensor();
    magnetometer = &lis3mdl;

    lsm6dsox.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    lsm6dsox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

    lsm6dsox.setAccelDataRate(LSM6DS_RATE_208_HZ);
    lsm6dsox.setGyroDataRate(LSM6DS_RATE_208_HZ);
    lis3mdl.setDataRate(LIS3MDL_DATARATE_1000_HZ);
    lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
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
    while (!gnss.begin())
    {
        Serial.println(F("[SENSORS] U-BLOX M10 GNSS not detected at default I2C address. Retrying..."));
        delay(3000);
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