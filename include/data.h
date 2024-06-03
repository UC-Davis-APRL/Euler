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
#include <nav.h>
#include <vehicle.h>

class Data
{
private:
    Sensors* sensors;
    Nav* nav;
    Vehicle* vehicle;
public:
    Data(Sensors* sensors, Nav* nav, Vehicle* vehicle) : sensors(sensors), nav(nav), vehicle(vehicle) {}
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