/*
    control.h

    Manages vehicle control logic

    Authors: Orfeas Magoulas
*/

#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>
#include <nav.h>
#include "actuators.h"
#include "guidance.h"
#include <ArduinoEigenDense.h>
#include <cmath>   // for isnan()
using namespace Eigen;

// *FIX*: Ensure PI is defined (Arduino may already define PI, but this is safe)
#ifndef PI
#define PI 3.14159265
#endif

// *FIX*: Define conversion macros
#define DEG_TO_RAD (PI/180.0)
#define RAD_TO_DEG (180.0/PI)

#define ALTITUDE_CONTROL_RATE_HZ 20
#define ATTITUDE_CONTROL_RATE_HZ 10
#define RCS_CONTROL_RATE_HZ 10

typedef Matrix<float, 3, 6> Matrix3x6;
typedef Matrix<float, 6, 1> Vector6;
typedef Matrix<float, 3, 1> Vector3c;

class Control
{
private:
    Nav *nav;
    Actuators *actuators;
    Guidance *guidance;

    bool debug = true;

    bool attitudeControl = false;
    bool altitudeControl = false;
    bool rcs = false;

    /*
        Attitude Control
    */
    Matrix3x6 k_matrix;

    // Runtime variable for control rate
    unsigned long attitudeTimestamp = 0;

    /*
        RCS Control
    */
    float Kp_RCS = 0.001f;
    float Kd_RCS = 0.1f;
    float prevErrorRCS = 0.0f;
    unsigned long rcsTimestamp = 0;

    /*
        Altitude Control
    */
    int motorSpeed = 0;
    float Kp_altitude = 80.0f;
    float Ki_altitude = 40.0f;
    float Kd_altitude = 0.0f;
    float targetHeight = 0.5f;

    // Runtime variables (for PID)
    float integralAltitude = 0.0f;
    float prevErrorAltitude = 0.0f;
    unsigned long altitudeTimestamp = 0;

public:
    Control(Nav *nav, Actuators *actuators, Guidance *guidance)
        : nav(nav), actuators(actuators), guidance(guidance) {}

    void init()
    {
        Serial.println(F("[CONTROL] Initializing..."));
        actuators->init();

        // *FIX*: The K matrix is defined row‐by‐row.
        k_matrix << 1.0000, 0.0000, 0.0000, 1.0954, 0.0000, 0.0000,
                    0.0000, 1.0000, 0.0000, 0.0000, 1.0954, 0.0000,
                    0.0000, 0.0000, 1.0000, 0.0000, 0.0000, 1.0954;

        Serial.println(F("[CONTROL] Control initialization complete!"));
    }

    void run()
    {
        if (altitudeControl)
        {
            this->runAltitudeController();
        }

        if (rcs)
        {
            this->runRCS();
        }

        if (attitudeControl)
        {
            this->runAttitudeController();
        }
        else
        {
            actuators->setPitchServoAngle(0);
            actuators->setRollServoAngle(0);
        }
    }

    void enableAltitudeControl(bool on_off)
    {
        if (on_off)
        {
            altitudeControl = true;
            altitudeTimestamp = micros();
            Serial.println(F("[CONTROL] Enabled altitude control."));
            return;
        }
        altitudeControl = false;
        Serial.println(F("[CONTROL] Disabled altitude control."));
    }

    void enableAttitudeControl(bool on_off)
    {
        if (on_off)
        {
            attitudeControl = true;
            attitudeTimestamp = micros();
            Serial.println(F("[CONTROL] Enabled attitude control."));
            return;
        }
        attitudeControl = false;
        Serial.println(F("[CONTROL] Disabled attitude control."));
    }

    void enableRCS(bool on_off)
    {
        if (on_off)
        {
            rcs = true;
            rcsTimestamp = micros();
            Serial.println(F("[CONTROL] Enabled RCS control."));
            return;
        }
        rcs = false;
        Serial.println(F("[CONTROL] Disabled RCS control."));
    }

    void runAltitudeController()
    {
        unsigned long currentTime = micros();
        unsigned long deltaTimeMicros = currentTime - altitudeTimestamp;

        if (deltaTimeMicros < (1000000UL / ALTITUDE_CONTROL_RATE_HZ))
        {
            return;
        }
        altitudeTimestamp = currentTime;
        float deltaTime = deltaTimeMicros / 1000000.0f; // seconds

        // Altitude PID control
        float errorAltitude = targetHeight - nav->height;

        // Update integral term with anti-windup
        integralAltitude += errorAltitude * deltaTime;
        integralAltitude = constrain(integralAltitude, 0.0f, 100.0f);

        // Compute derivative term
        float derivativeAltitude = (errorAltitude - prevErrorAltitude) / deltaTime;

        // Compute control output
        float controlAltitude = Kp_altitude * errorAltitude +
                                Ki_altitude * integralAltitude +
                                Kd_altitude * derivativeAltitude;

        prevErrorAltitude = errorAltitude;

        int desiredChange = static_cast<int>(controlAltitude);
        motorSpeed = 40 + desiredChange;
        motorSpeed = constrain(motorSpeed, 0, 255);

        Serial.printf("Height: %.2f, Error: %.2f, Control Output: %.2f, Motor Setpoints: %d\n",
                      nav->height, errorAltitude, controlAltitude, motorSpeed);

        actuators->setThrust(motorSpeed / 255.0);
    }

    void runRCS()
    {
        unsigned long currentTime = micros();
        unsigned long deltaTimeMicros = currentTime - rcsTimestamp;

        if (deltaTimeMicros < (1000000UL / RCS_CONTROL_RATE_HZ))
        {
            return;
        }
        rcsTimestamp = currentTime;
        float deltaTime = deltaTimeMicros / 1000000.0f;

        // Here we assume nav->yaw is already in radians.
        float r = nav->yaw;
        float errorRCS = 0 - r;

        float derivativeRCS = (errorRCS - prevErrorRCS) / deltaTime;
        float controlRCS = Kp_RCS * errorRCS + Kd_RCS * derivativeRCS;

        Serial.printf("r: %.2f, control: %.2f\n", r, errorRCS * Kp_RCS);
        actuators->setTorque(errorRCS * Kp_RCS);
    }

    void runAttitudeController()
    {
        unsigned long currentTime = micros();
        unsigned long deltaTimeMicros = currentTime - attitudeTimestamp;

        if (deltaTimeMicros < (1000000UL / ATTITUDE_CONTROL_RATE_HZ))
        {
            return;
        }
        attitudeTimestamp = currentTime;

        float roll  = nav->roll  * DEG_TO_RAD;   // phi
        float pitch = nav->pitch * DEG_TO_RAD;   // theta
        float yaw   = nav->yaw   * DEG_TO_RAD;   // psi
        float p     = nav->p     * DEG_TO_RAD;
        float q     = nav->q     * DEG_TO_RAD;
        float r     = nav->r     * DEG_TO_RAD;

        // Optional: Check for invalid sensor values.
        if (isnan(roll) || isnan(pitch) || isnan(yaw) ||
            isnan(p) || isnan(q) || isnan(r))
        {
            Serial.println(F("[CONTROL] Invalid sensor data."));
            return;
        }

        Vector6 state;
        Vector3c output;
        state << roll, pitch, yaw, p, q, r;
        // Note: The control law is: output = -K * state.
        Vector6 setpoint;
        setpoint << 3 * DEG_TO_RAD, 0 * DEG_TO_RAD, 0 * DEG_TO_RAD, 0, 0, 0;
        output = k_matrix * (setpoint - state);

        if (debug)
        {
            Serial.print("Roll: ");
            Serial.print(roll);
            Serial.print(" Pitch: ");
            Serial.print(pitch);
            Serial.print(" Yaw: ");
            Serial.println(yaw);

            // Serial.print("p: ");
            // Serial.print(p);
            // Serial.print(" q: ");
            // Serial.print(q);
            // Serial.print(" r: ");
            // Serial.println(r);
        }
        const float gain = 1.0f;

        double weight = 3 * 9.81; // Craft weight in Newtons
        double lever_arm = 0.2;   // Distance from COM to gimbal (meters)

        // *FIX*: Compute the normalized command and convert to degrees.
        float norm_roll = output[0] / (weight * lever_arm);
        norm_roll = constrain(norm_roll, (-PI / 2) + 0.01, (PI / 2) - 0.01);
        double angle_roll = norm_roll * RAD_TO_DEG;

        float norm_pitch = output[1] / (weight * lever_arm);
        norm_pitch = constrain(norm_pitch, (-PI / 2) + 0.01, (PI / 2) - 0.01);
        double angle_pitch = norm_pitch * RAD_TO_DEG;

        // Serial.println(angle_roll);
        // Serial.println(angle_pitch);

        // Finally, command the servos (limiting the angle to [-30, 30] degrees)
        actuators->setPitchServoAngle(constrain(-angle_pitch * gain, -30.0f, 30.0f));
        actuators->setRollServoAngle(constrain(-angle_roll * gain, -30.0f, 30.0f));
    }
};

#endif // CONTROL_H