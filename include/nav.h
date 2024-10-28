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
#define INTEGRAL_UPDATE_RATE_HZ 100

// #define AHRS_DEBUG_OUTPUT

class Nav
{
private:
    Sensors *sensors;
    int filterTimestamp;
    int imuTimestamp;
    int gnssTimestamp;
    unsigned long integralTimestamp;

    void updateFilter();
    void updateIMU();
    void updateGNSS();

    // Gravity vector components
    float gX, gY, gZ;

    // EKF variables
    float x[6];         // State vector: [px, py, pz, vx, vy, vz]
    float P[6][6];      // Covariance matrix
    float Q[6][6];      // Process noise covariance
    float R[3][3];      // Measurement noise covariance
    unsigned long previousTime;

    void predict(float dt);
    void updateWithGNSS();
    void latLonToXY(float lat, float lon, float *x, float *y);

    // Reference coordinates for local frame
    float refLat;
    float refLon;
    bool refSet;

public:
    // IMU
    float roll, pitch, heading;
    float qw, qx, qy, qz;
    float ax, ay, az;

    // GNSS
    float lat, lon, alt;
    int gnss_fix_type;
    int gnss_satellites;

    float heightRaw;

    // Velocity & Position (Filtered by EKF)
    float vx, vy, vz;
    float px, py, pz;

    Nav(Sensors *sensors)
        : sensors(sensors), filterTimestamp(0), imuTimestamp(0), gnssTimestamp(0), integralTimestamp(0),
          gX(0.0f), gY(0.0f), gZ(0.0f), previousTime(0),
          refLat(0.0f), refLon(0.0f), refSet(false),
          vx(0.0f), vy(0.0f), vz(0.0f), px(0.0f), py(0.0f), pz(0.0f) {}

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
    unsigned long stableStartTime = 0;
    bool stable = false;
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
    Serial.printf("Pitch: %f, Roll: %f\n", pitch, roll);
    Serial.println(F("[NAV] Convergence complete!"));

    // Implement calibration after the convergence stage
    Serial.println(F("[NAV] Starting calibration to eliminate drift..."));

    const unsigned long calibrationDuration = 5000; // 5 seconds
    unsigned long calibrationStartTime = millis();
    unsigned int calibrationSamples = 0;
    float sumAx = 0.0f, sumAy = 0.0f, sumAz = 0.0f;

    while (millis() - calibrationStartTime < calibrationDuration)
    {
        updateFilter();
        updateIMU();

        // Accumulate accelerometer readings
        sumAx += ax;
        sumAy += ay;
        sumAz += az;
        calibrationSamples++;

        delay(10); // Small delay to prevent overwhelming the loop
    }

    // Calculate average gravity vector
    gX = sumAx / calibrationSamples;
    gY = sumAy / calibrationSamples;
    gZ = sumAz / calibrationSamples;

    Serial.println(F("[NAV] Calibration complete!"));
    Serial.print("Calibration Vector - gX: ");
    Serial.print(gX);
    Serial.print(", gY: ");
    Serial.print(gY);
    Serial.print(", gZ: ");
    Serial.println(gZ);

    Serial.println(F("[NAV] Initializing GNSS/ Position Extended Kalman Filter"));

    Serial.println(F("[NAV] Initializing State Vectors"));

    // Initialize EKF variables
    // Initialize state vector x
    for (int i = 0; i < 6; i++)
    {
        x[i] = 0.0f;
    }
    Serial.println(F("[NAV] Initializing Covariance Matrix P"));

    // Initialize covariance matrix P to identity times a small value
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            P[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    Serial.println(F("[NAV] Initializing Process Noise Covariance Matrix Q"));

    // Initialize process noise covariance Q
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            Q[i][j] = 0.0f;
        }
    }
    Q[0][0] = Q[1][1] = Q[2][2] = 0.1f;  // Position process noise
    Q[3][3] = Q[4][4] = Q[5][5] = 0.1f;  // Velocity process noise

    Serial.println(F("[NAV] Initializing Measurement Noise Covariance Matrix R"));
    // Initialize measurement noise covariance R
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            R[i][j] = 0.0f;
        }
    }
    R[0][0] = R[1][1] = 5.0f; // GNSS horizontal position measurement noise
    R[2][2] = 10.0f;          // GNSS vertical position measurement noise

    Serial.println(F("[NAV] EKF Initialization Complete"));

    // Initialize previousTime
    previousTime = micros();

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

    // Ignore this
    sensors->updateUltrasonic();
    heightRaw = sensors->heightRaw;
    // Will delete

    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    filter.getQuaternion(&qw, &qx, &qy, &qz);
    filter.getLinearAcceleration(&ax, &ay, &az);

    const float GRAVITY = 9.80665f;
    ax *= GRAVITY;
    ay *= GRAVITY;
    az *= GRAVITY;

    // Subtract gravity vector to get true linear acceleration
    ax -= gX;
    ay -= gY;
    az -= gZ;
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

    if (gnss_fix_type >= 2) // Assuming fix_type >= 2 means we have valid data
    {
        lat = sensors->gnss.getLatitude() / 1e7;
        lon = sensors->gnss.getLongitude() / 1e7;
        alt = sensors->gnss.getAltitudeMSL() / 1000.0;

        if (!refSet)
        {
            refLat = lat;
            refLon = lon;
            refSet = true;
            Serial.println(F("[NAV] Reference location set."));
        }
    }
}

/*
    Convert latitude and longitude to local Cartesian coordinates
*/
inline void Nav::latLonToXY(float lat, float lon, float *x, float *y)
{
    // Simple equirectangular approximation
    const float degToRad = 3.14159265358979323846f / 180.0f;
    float dLat = (lat - refLat) * degToRad;
    float dLon = (lon - refLon) * degToRad;
    float latRad = refLat * degToRad;

    const float R = 6371000.0f; // Earth radius in meters

    *x = R * dLon * cos(latRad);
    *y = R * dLat;
}

/*
    EKF Prediction Step
*/
inline void Nav::predict(float dt)
{
    // Update the state vector x
    // x[0] = px
    // x[1] = py
    // x[2] = pz
    // x[3] = vx
    // x[4] = vy
    // x[5] = vz

    // Predict position
    x[0] += x[3] * dt + 0.5f * ax * dt * dt;
    x[1] += x[4] * dt + 0.5f * ay * dt * dt;
    x[2] += x[5] * dt + 0.5f * az * dt * dt;

    // Predict velocity
    x[3] += ax * dt;
    x[4] += ay * dt;
    x[5] += az * dt;

    // State transition matrix F
    float F[6][6] = {
        {1, 0, 0, dt, 0, 0},
        {0, 1, 0, 0, dt, 0},
        {0, 0, 1, 0, 0, dt},
        {0, 0, 0, 1, 0, 0},
        {0, 0, 0, 0, 1, 0},
        {0, 0, 0, 0, 0, 1}};

    // Compute P = F * P * F^T + Q
    // First, compute F * P
    float FP[6][6];
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            FP[i][j] = 0.0f;
            for (int k = 0; k < 6; k++)
            {
                FP[i][j] += F[i][k] * P[k][j];
            }
        }
    }

    // Then compute P = FP * F^T + Q
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            P[i][j] = 0.0f;
            for (int k = 0; k < 6; k++)
            {
                P[i][j] += FP[i][k] * F[j][k]; // Note F^T[j][k] = F[k][j]
            }
            P[i][j] += Q[i][j];
        }
    }
}

/*
    EKF Update Step with GNSS Data
*/
inline void Nav::updateWithGNSS()
{
    // Measurement vector z
    float px_meas, py_meas;
    latLonToXY(lat, lon, &px_meas, &py_meas);
    float pz_meas = alt; // Assuming alt is in meters

    float z[3] = {px_meas, py_meas, pz_meas};

    // Measurement matrix H
    float H[3][6] = {
        {1, 0, 0, 0, 0, 0}, // Position x
        {0, 1, 0, 0, 0, 0}, // Position y
        {0, 0, 1, 0, 0, 0}  // Position z
    };

    // Compute innovation y = z - H * x
    float Hx[3];
    for (int i = 0; i < 3; i++)
    {
        Hx[i] = 0.0f;
        for (int j = 0; j < 6; j++)
        {
            Hx[i] += H[i][j] * x[j];
        }
    }

    float y[3];
    for (int i = 0; i < 3; i++)
    {
        y[i] = z[i] - Hx[i];
    }

    // Compute S = H * P * H^T + R
    float HP[3][6];
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            HP[i][j] = 0.0f;
            for (int k = 0; k < 6; k++)
            {
                HP[i][j] += H[i][k] * P[k][j];
            }
        }
    }

    float S[3][3];
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            S[i][j] = 0.0f;
            for (int k = 0; k < 6; k++)
            {
                S[i][j] += HP[i][k] * H[j][k]; // Note H^T[j][k] = H[k][j]
            }
            S[i][j] += R[i][j];
        }
    }

    // Compute Kalman gain K = P * H^T * S^{-1}
    float PHt[6][3];
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            PHt[i][j] = 0.0f;
            for (int k = 0; k < 6; k++)
            {
                PHt[i][j] += P[i][k] * H[j][k]; // H^T[j][k] = H[k][j]
            }
        }
    }

    // Compute S^{-1} (inverse of 3x3 matrix)
    float detS = S[0][0] * (S[1][1] * S[2][2] - S[1][2] * S[2][1]) -
                 S[0][1] * (S[1][0] * S[2][2] - S[1][2] * S[2][0]) +
                 S[0][2] * (S[1][0] * S[2][1] - S[1][1] * S[2][0]);

    if (fabs(detS) < 1e-6)
    {
        // Singular matrix, cannot invert
        Serial.println(F("[NAV] Singular matrix in EKF update."));
        return;
    }

    float invDetS = 1.0f / detS;

    float S_inv[3][3];
    S_inv[0][0] = (S[1][1] * S[2][2] - S[1][2] * S[2][1]) * invDetS;
    S_inv[0][1] = (S[0][2] * S[2][1] - S[0][1] * S[2][2]) * invDetS;
    S_inv[0][2] = (S[0][1] * S[1][2] - S[0][2] * S[1][1]) * invDetS;
    S_inv[1][0] = (S[1][2] * S[2][0] - S[1][0] * S[2][2]) * invDetS;
    S_inv[1][1] = (S[0][0] * S[2][2] - S[0][2] * S[2][0]) * invDetS;
    S_inv[1][2] = (S[0][2] * S[1][0] - S[0][0] * S[1][2]) * invDetS;
    S_inv[2][0] = (S[1][0] * S[2][1] - S[1][1] * S[2][0]) * invDetS;
    S_inv[2][1] = (S[0][1] * S[2][0] - S[0][0] * S[2][1]) * invDetS;
    S_inv[2][2] = (S[0][0] * S[1][1] - S[0][1] * S[1][0]) * invDetS;

    // Compute K = P * H^T * S^{-1}
    float K[6][3];
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            K[i][j] = 0.0f;
            for (int k = 0; k < 3; k++)
            {
                K[i][j] += PHt[i][k] * S_inv[k][j];
            }
        }
    }

    // Update state x = x + K * y
    for (int i = 0; i < 6; i++)
    {
        float correction = 0.0f;
        for (int j = 0; j < 3; j++)
        {
            correction += K[i][j] * y[j];
        }
        x[i] += correction;
    }

    // Update covariance P = (I - K * H) * P
    float KH[6][6];
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            KH[i][j] = 0.0f;
            for (int k = 0; k < 3; k++)
            {
                KH[i][j] += K[i][k] * H[k][j];
            }
        }
    }

    // Compute (I - K * H)
    float I_KH[6][6];
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            I_KH[i][j] = (i == j ? 1.0f : 0.0f) - KH[i][j];
        }
    }

    // Update P = (I - K * H) * P
    float newP[6][6];
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            newP[i][j] = 0.0f;
            for (int k = 0; k < 6; k++)
            {
                newP[i][j] += I_KH[i][k] * P[k][j];
            }
        }
    }

    // Copy newP to P
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            P[i][j] = newP[i][j];
        }
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

    unsigned long now = micros();
    float dt = (now - previousTime) / 1e6f;
    if (dt > 0.1f)
    {
        dt = 0.1f; // Limit dt to prevent large jumps
    }
    previousTime = now;

    // Perform EKF prediction step
    predict(dt);

    // If GNSS data is available and reference is set, perform EKF update step
    if (gnss_fix_type >= 2 && refSet)
    {
        updateWithGNSS();
    }

    // Update public variables with the EKF estimates
    px = x[0];
    py = x[1];
    pz = x[2];
    vx = x[3];
    vy = x[4];
    vz = x[5];
}

#endif
