/*
    vehicle.h

    Manages vehicle states
    WIP

    @authors Orfeas Magoulas
*/

#ifndef VEHICLE_H
#define VEHICLE_H

class Vehicle
{
private:
public:
    Vehicle() {}
    void init();
    void update();
};

inline void Vehicle::init()
{
    Serial.println(F("[VEHICLE] Initializing..."));
    // TODO
    Serial.println(F("[VEHICLE] Initialization complete!"));
}

inline void Vehicle::update()
{
}

#endif