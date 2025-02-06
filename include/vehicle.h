#ifndef VEHICLE_H
#define VEHICLE_H

#include <Arduino.h>
#include <guidance.h>
#include <control.h>

/*
    Vehicle Class
    Manages vehicle states and sequences, including Sequence A, B, and C.

    Dependencies:
    - Control: For managing vehicle controls like altitude and attitude.

    Author: Orfeas Magoulas
*/

class Vehicle
{
public:
    Vehicle(Guidance* guidance, Control* control, Actuators* actuators);
    void init();
    void update();
    void startSequenceA();
    void stopSequenceA();
    void startSequenceB();
    void stopSequenceB();
    void startSequenceC();  // <-- Added for Sequence C
    void stopSequenceC();   // <-- Added for Sequence C

private:
    enum SequenceAState {
        SEQ_A_IDLE,
        SEQ_A_START,
        SEQ_A_MAIN,
        SEQ_A_END,
        SEQ_A_COMPLETE
    };

    enum SequenceBState {
        SEQ_B_IDLE,
        SEQ_B_START,
        SEQ_B_WAIT,          // <-- New State Added
        SEQ_B_SET_THRUST,
        SEQ_B_INCREASE_TORQUE,
        SEQ_B_DECREASE_TORQUE,
        SEQ_B_END,
        SEQ_B_COMPLETE
    };

    // <-- Added Sequence C States
    enum SequenceCState {
        SEQ_C_IDLE,
        SEQ_C_START,
        SEQ_C_MAIN,
        SEQ_C_END,
        SEQ_C_COMPLETE
    };

    SequenceAState sequenceAState;
    SequenceBState sequenceBState;
    SequenceCState sequenceCState;  // <-- Added for Sequence C

    unsigned long sequenceATimer;
    unsigned long sequenceBTimer;
    unsigned long sequenceCTimer;  // <-- Added for Sequence C

    Guidance* guidance;
    Control* control;
    Actuators* actuators;

    void sequenceA_run();
    void sequenceB_run();
    void sequenceC_run();  // <-- Added for Sequence C
};

// Constructor
inline Vehicle::Vehicle(Guidance* guidance, Control* control, Actuators* actuators)
    : guidance(guidance), control(control), actuators(actuators),
      sequenceAState(SEQ_A_IDLE), sequenceBState(SEQ_B_IDLE), sequenceCState(SEQ_C_IDLE), // <-- Initialize Sequence C
      sequenceATimer(0), sequenceBTimer(0), sequenceCTimer(0) // <-- Initialize Sequence C Timer
{
}

inline void Vehicle::init()
{
    Serial.println(F("[VEHICLE] Initializing..."));
    Serial.println(F("[VEHICLE] Initialization complete!"));
}

inline void Vehicle::update()
{
    sequenceA_run();
    sequenceB_run();
    sequenceC_run();
}

inline void Vehicle::startSequenceA()
{
    if (sequenceAState == SEQ_A_IDLE) {
        sequenceAState = SEQ_A_START;
    }
}

inline void Vehicle::stopSequenceA()
{
    sequenceAState = SEQ_A_IDLE;
}

inline void Vehicle::startSequenceB()
{
    if (sequenceBState == SEQ_B_IDLE) {
        sequenceBState = SEQ_B_START;
    }
}

inline void Vehicle::stopSequenceB()
{
    sequenceBState = SEQ_B_IDLE;
}

inline void Vehicle::startSequenceC()
{
    if (sequenceCState == SEQ_C_IDLE) {
        sequenceCState = SEQ_C_START;
    }
}

inline void Vehicle::stopSequenceC()
{
    sequenceCState = SEQ_C_IDLE;
}

inline void Vehicle::sequenceA_run()
{
    switch (sequenceAState) {
        case SEQ_A_IDLE:
            break;

        case SEQ_A_START:
            Serial.println(F("[SEQUENCE A] Starting sequence A"));
            Serial.println(F("[SEQUENCE A] 10 seconds until run..."));
            sequenceATimer = millis();
            sequenceAState = SEQ_A_MAIN;
            break;

        case SEQ_A_MAIN:
            if (millis() - sequenceATimer >= 10000) {
                // control->enableAltitudeControl(true);
                actuators->setThrust(.5);
                Serial.println(F("[SEQUENCE A] Altitude control enabled"));
                sequenceATimer = millis();
                sequenceAState = SEQ_A_END;
            }
            break;

        case SEQ_A_END:
            if (millis() - sequenceATimer >= -1) {
                control->enableAltitudeControl(false);
                actuators->disarm();
                sequenceAState = SEQ_A_COMPLETE;
                Serial.println(F("[SEQUENCE A] Sequence A complete"));
            }
            break;

        case SEQ_A_COMPLETE:
            break;
    }
}

inline void Vehicle::sequenceB_run()
{
    switch (sequenceBState) {
        case SEQ_B_IDLE:
            break;

        case SEQ_B_START:
            Serial.println(F("[SEQUENCE B] Starting sequence B"));
            Serial.println(F("[SEQUENCE B] 10 seconds until run..."));
            sequenceBTimer = millis();
            sequenceBState = SEQ_B_WAIT;
            break;

        case SEQ_B_WAIT:
            if (millis() - sequenceBTimer >= 6000) {
                Serial.println(F("[SEQUENCE B] Wait complete. Proceeding with sequence B actions."));
                sequenceBState = SEQ_B_SET_THRUST;
            }
            break;

        case SEQ_B_SET_THRUST:
            Serial.println(F("[SEQUENCE B] Setting thrust to 50%"));
            actuators->setThrust(0.7);
            sequenceBTimer = millis();
            sequenceBState = SEQ_B_INCREASE_TORQUE;
            break;

        case SEQ_B_INCREASE_TORQUE:
            if (millis() - sequenceBTimer >= 4000) {
                Serial.println(F("[SEQUENCE B] Increasing torque to +0.1"));
                actuators->setTorque(0.5);
                sequenceBTimer = millis();
                sequenceBState = SEQ_B_DECREASE_TORQUE;
            }
            break;

        case SEQ_B_DECREASE_TORQUE:
            if (millis() - sequenceBTimer >= 4000) {
                Serial.println(F("[SEQUENCE B] Decreasing torque to -0.1"));
                actuators->setTorque(-0.5);
                sequenceBTimer = millis();
                sequenceBState = SEQ_B_END;
            }
            break;

        case SEQ_B_END:
            if (millis() - sequenceBTimer >= 4000) {
                Serial.println(F("[SEQUENCE B] Resetting torque to 0 and stopping motors"));
                actuators->setTorque(0.0);
                actuators->setThrust(0.0);
                actuators->disarm();
                sequenceBState = SEQ_B_COMPLETE;
                Serial.println(F("[SEQUENCE B] Sequence B complete"));
            }
            break;

        case SEQ_B_COMPLETE:
            break;
    }
}

inline void Vehicle::sequenceC_run()
{
    switch (sequenceCState) {
        case SEQ_C_IDLE:
            break;

        case SEQ_C_START:
            Serial.println(F("[SEQUENCE C] Starting sequence C"));
            Serial.println(F("[SEQUENCE C] 10 seconds until run..."));
            sequenceCTimer = millis();
            sequenceCState = SEQ_C_MAIN;
            break;

        case SEQ_C_MAIN:
            if (millis() - sequenceCTimer >= 10000) {
                Serial.println(F("[SEQUENCE C] Executing main actions of sequence C"));
                // control->enableRCS(true);
                actuators->setThrust(.67);
                sequenceCTimer = millis();
                sequenceCState = SEQ_C_END;
            }
            break;

        case SEQ_C_END:
            if (millis() - sequenceCTimer >= 1000) {
                Serial.println(F("[SEQUENCE C] Ending sequence C"));
                sequenceCState = SEQ_C_COMPLETE;
                Serial.println(F("[SEQUENCE C] Sequence C complete"));
            }
            break;

        case SEQ_C_COMPLETE:
            break;
    }
}

#endif // VEHICLE_H
