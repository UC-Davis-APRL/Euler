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

Comms comms = Comms(&sensors, &vehicle, &nav);
Data data = Data(&sensors, &vehicle, &nav);

enum SequenceAState {
    SEQ_A_IDLE,
    SEQ_A_START,
    SEQ_A_DELAY1,
    SEQ_A_TEST_ROTORS,
    SEQ_A_DELAY2,
    SEQ_A_COMPLETE
};

SequenceAState sequenceAState = SEQ_A_IDLE;
unsigned long sequenceATimer = 0;

void sequenceA_run() {
    switch (sequenceAState) {
        case SEQ_A_IDLE:
            // Waiting to start
            break;

        case SEQ_A_START:
            Serial.println(F("[SEQUENCE A] Starting sequence A"));
            sequenceATimer = millis();
            sequenceAState = SEQ_A_DELAY1;
            break;

        case SEQ_A_DELAY1:
            if (millis() - sequenceATimer >= 6000) {
                Serial.println(F("[SEQUENCE A] Testing rotor A .5 speed"));
                control.altitudeControl(true);
                // control.setMotor1SpeedTest(88);
                // control.setMotor2SpeedTest(118);
  
                sequenceATimer = millis();
                sequenceAState = SEQ_A_DELAY2;
            }
            break;

        case SEQ_A_DELAY2:
            if (millis() - sequenceATimer >= 20000) {
                control.setMotor1SpeedTest(0);
                control.setMotor2SpeedTest(0);
                control.altitudeControl(false);
                Serial.println(F("[SEQUENCE A] Sequence A complete"));
                sequenceAState = SEQ_A_COMPLETE;
            }
            break;

        case SEQ_A_COMPLETE:
            // Sequence complete, reset if needed
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

    comms.init();
    data.init();

    Serial.println(F("[MAIN] Initialization complete!"));

    control.arm();
    delay(6000);
    sequenceAState = SEQ_A_START;
    // sequenceBState = SEQ_B_START;
}

void loop()
{
    vehicle.update();
    nav.run();
    guidance.run();
    control.run();

    comms.run();
    data.log();

    sequenceA_run();
}