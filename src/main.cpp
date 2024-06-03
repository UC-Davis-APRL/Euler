#include <Arduino.h>
#include <TeensyThreads.h>
#include <sensors.h>

#include <nav.h>
#include <guidance.h>
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

/*
    Low priority thread, less CPU time, yields to primary loop.
*/
void lowPriority() {
    while(true) {
        comms.run();
        data.log();
        threads.delay(10);
    }
}

void setup() {
    Serial.begin(9600);
    while (!Serial) yield();

    Wire.begin();
    Wire.setClock(400000);

    sensors.init();
    vehicle.init();
    
    nav.init();
    guidance.init();
    control.init();

    comms.init();
    data.init();

    threads.addThread(lowPriority);
}

void loop() {
    vehicle.update();

    nav.run();
    guidance.run();
    control.run();
}