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
    Sensors* sensors;
    Nav* nav;
    Vehicle* vehicle;
public:
    Data(Sensors* sensors, Vehicle* vehicle, Nav* nav) : sensors(sensors), vehicle(vehicle), nav(nav) {}
    void init();
    void log();
};

inline void Data::init()
{
    
}

inline void Data::log()
{

}

#endif