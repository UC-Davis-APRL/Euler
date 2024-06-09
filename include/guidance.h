/*
    guidance.h
    
    Manages vehicle guidance
    WIP
    
    @authors Orfeas Magoulas
*/

#ifndef GUIDANCE_H
#define GUIDANCE_H

#include <vehicle.h>
#include <nav.h>

class Guidance
{
private:
    Vehicle* vehicle;
    Nav* nav;
public:
    Guidance(Vehicle* vehicle, Nav* nav) : vehicle(vehicle), nav(nav)  {}
    void init();
    void run();
};

inline void Guidance::init()
{
    Serial.println(F("[GUIDANCE] Initializing..."));
    // TODO
    Serial.println(F("[GUIDANCE] Initialization complete!"));
}

inline void Guidance::run()
{
    
}

#endif