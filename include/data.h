/*
    data.h

    Handles onboard logging & flight data
    Stores to SD card
    WIP

    @authors Orfeas Magoulas
*/

#ifndef DATA_H
#define DATA_H

#include <sensors.h>
#include <vehicle.h>
#include <nav.h>

class Data
{
private:
    Sensors *sensors;
    Vehicle *vehicle;
    Nav *nav;

public:
    Data(Sensors *sensors, Vehicle *vehicle, Nav *nav) : sensors(sensors), vehicle(vehicle), nav(nav) {}
    void init();
    void log();
};

inline void Data::init()
{
    Serial.println(F("[DATA] Initializing..."));
    // TODO
    Serial.println(F("[DATA] Initialization complete!"));
}

inline void Data::log()
{
}

#endif