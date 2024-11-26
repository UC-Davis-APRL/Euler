#include <Arduino.h>
#include <sensors.h>
#include <nav.h>
#include <guidance.h>
#include <control.h>
#include <comms.h>
#include <data.h>
#include <Servo.h>

/*
    Modules (enable or disable by commenting out)
    - Sensors: Provides access to IMU, GNSS
    - Vehicle: Provides and manages vehicle state
    - Nav: Fuses raw sensor data into stable navigational values
    - Guidance: Handles vehicle guidance
    - Control: Actuates vehicle control surfaces
    - Comms: Handles bidirectional radio link
    - Data: Handles onboard logging & flight data
*/
Sensors sensors = Sensors();
Vehicle vehicle = Vehicle();

Nav nav = Nav(&sensors);
Guidance guidance = Guidance(&vehicle, &nav);
Control control = Control(&vehicle, &nav, &guidance);

// Comms comms = Comms(&sensors, &vehicle, &nav);
Data data = Data(&sensors, &vehicle, &nav);

enum SequenceAState {
    SEQ_A_IDLE,
    SEQ_A_START,
    SEQ_A_MAIN,
    SEQ_A_END,
    SEQ_A_COMPLETE
};

SequenceAState sequenceAState = SEQ_A_IDLE;
unsigned long sequenceATimer = 0;

void sequenceA_run() {
    switch (sequenceAState) {
        case SEQ_A_IDLE:
            break;

        case SEQ_A_START:
            Serial.println(F("[SEQUENCE A] Starting sequence A"));
            sequenceATimer = millis();
            sequenceAState = SEQ_A_MAIN;
            break;

        case SEQ_A_MAIN:
            if (millis() - sequenceATimer >= 5000) {
                control.enableAltitudeControl(true);
                sequenceATimer = millis();
                sequenceAState = SEQ_A_END;
            }
            break;

        case SEQ_A_END:
            if (millis() - sequenceATimer >= 15000) {
                control.enableAltitudeControl(false);
                control.arm(false);
                sequenceAState = SEQ_A_COMPLETE;
                Serial.println(F("[SEQUENCE A] Sequence A complete"));
            }
            break;

        case SEQ_A_COMPLETE:
            break;
    }
}

void setup()
{
    Serial.begin(9600);
    while (!Serial)
        yield();
    Serial.println(F("[MAIN] Initializing..."));

    Wire.begin();
    Wire.setClock(400000);

    sensors.init();
    vehicle.init();

    nav.init();
    guidance.init();
    control.init();

    // comms.init();
    data.init();

    Serial.println(F("[MAIN] Initialization complete!"));

    control.enableAttitudeControl(true);
    control.enableAltitudeControl(true);
    // control.arm(true);
    // delay(5000);
    // sequenceAState = SEQ_A_START;
}

void loop()
{
    vehicle.update();
    nav.run();
    guidance.run();
    control.run();

    // comms.run();
    data.log();

    sequenceA_run();
}