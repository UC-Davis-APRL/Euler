#include <Arduino.h>
#include <TeensyThreads.h>
#include <sensors.h>
#include <nav.h>
#include <comms.h>
#include <data.h>

/*
    Modules (enable or disable by commenting out)
    - Sensors: Provides access to IMU, GNSS
    - Nav: Fuses raw sensor data into stable navigational values
    - Vehicle: Provides and manages vehicle state
    - Comms: Handles bidirectional MavLink communication
    - Data: Handles onboard logging & flight data
*/
Sensors sensors = Sensors();
Nav nav = Nav(&sensors);
Vehicle vehicle = Vehicle();
Comms comms = Comms(&sensors, &nav, &vehicle);
Data data = Data(&sensors, &nav, &vehicle);

/*
    Low priority thread, less CPU time, yields to primary loop.
*/
void lowPriority() {
    while(true) {
        comms.run();
        threads.delay(50);
    }
}

void setup() {
    Serial.begin(9600);
    while (!Serial) yield();

    Wire.begin();
    Wire.setClock(400000);

    sensors.init();
    nav.init();
    vehicle.init();
    comms.init();
    data.init();

    threads.addThread(lowPriority);
}

void loop() {
    nav.run();
    data.log();
}