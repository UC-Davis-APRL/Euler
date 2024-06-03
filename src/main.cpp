#include <Arduino.h>

#include <sensors.h>
#include <nav.h>
#include <comms.h>
#include <vehicle.h>

Sensors sensors = Sensors();
Nav nav = Nav(&sensors);
Vehicle vehicle = Vehicle();
Comms comms = Comms(&sensors, &nav, &vehicle);

void setup() {
    Serial.begin(9600);
    while (!Serial) yield();

    Wire.begin();
    Wire.setClock(400000);

    sensors.init();
    nav.init();
    vehicle.init();
    comms.init();
}

void loop() {
    nav.run();
    comms.run();
}