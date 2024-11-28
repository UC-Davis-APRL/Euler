#include <Arduino.h>
#include <sensors.h>
#include <nav.h>
#include <guidance.h>
#include <actuators.h>
#include <control.h>
#include <comms.h>
#include <data.h>

/*
    Modules (enable or disable by commenting out)
    - Sensors: Provides access to IMU, GNSS
    - Vehicle: Provides and manages vehicle state
    - Nav: Fuses raw sensor data into stable navigational values
    - Guidance: Handles vehicle guidance
    - Control: Actuates vehicle control surfaces
    - Actuators: Manages servos and motors
    - Comms: Handles bidirectional radio link
    - Data: Handles onboard logging & flight data
*/

Actuators actuators;

Sensors sensors = Sensors();
Nav nav = Nav(&sensors);
Guidance guidance = Guidance(&nav);

Control control = Control(&nav, &actuators, &guidance);

Vehicle vehicle = Vehicle(&guidance, &control, &actuators);

void setup()
{
    Serial.begin(9600);
    while (!Serial)
        yield();
    Serial.println(F("[MAIN] Initializing..."));

    sensors.init();

    actuators.init(); // Initialize Actuators before Control

    nav.init();
    guidance.init();
    control.init();

    vehicle.init();

    Serial.println(F("[MAIN] Initialization complete!"));

    control.enableAttitudeControl(false);
    
    actuators.arm();
    Serial.println(F("[MAIN] Armed. Starting in 10 seconds!"));
    control.enableRCS(true);

    vehicle.startSequenceA();
}

void loop()
{
    nav.run();
    guidance.run();
    control.run();

    vehicle.update();
}
