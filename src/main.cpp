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

void sequenceA() {
    // control.arm();
    Serial.println(F("[SEQUENCE A] Starting sequence A"));
    delay(6000);
    Serial.println(F("[SEQUENCE A] Testing rotor A .5 speed"));
    control.setMotor1SpeedTest(128); // Bottom, causes clockwise torque
    control.setMotor2SpeedTest(158); // Top, causes counter-clockwise torque
    delay(10000);
    control.setMotor1SpeedTest(0);
    control.setMotor2SpeedTest(0);
    Serial.println(F("[SEQUENCE A] Sequence A complete"));
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
}

void loop()
{
    vehicle.update();
    nav.run();
    guidance.run();
    control.run();

    comms.run();
    data.log();
}