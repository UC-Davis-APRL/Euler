#include <Arduino.h>
#include <sensors.h>
#include <nav.h>
#include <guidance.h>
#include <actuators.h>
#include <control.h>
#include <vehicle.h>

/*
    Modules (enable or disable by commenting out)
    - Sensors: Provides access to IMU, GNSS
    - Actuators: Manages servos and motors
    - Nav: Fuses raw sensor data into stable navigational values
    - Guidance: Handles vehicle guidance
    - Control: Actuates vehicle control surfaces
    - Vehicle: Handles state machines
    - Comms: Handles bidirectional radio link
    - Data: Handles onboard logging & flight data
*/

// Sensors & Actuators
auto sensors = Sensors();
auto actuators = Actuators();

// Guidance, Navigation & Control Logic
auto nav = Nav(&sensors);
auto guidance = Guidance(&nav);
auto control = Control(&nav, &actuators, &guidance);

// State Machines
auto vehicle = Vehicle(&guidance, &control, &actuators);

void setup()
{
    Serial.begin(9600);
    while (!Serial)
        yield();
    Serial.println(F("[MAIN] Initializing..."));

    sensors.init();
    actuators.init();

    nav.init();
    guidance.init();
    control.init();

    vehicle.init();

    Serial.println(F("[MAIN] Initialization complete!"));

    control.enableAttitudeControl(true);
    // control.enableRCS(true);
    // actuators.arm();
    // vehicle.startSequenceC();
}

void loop()
{
    nav.run();
    guidance.run();
    control.run();

    vehicle.update();
}
