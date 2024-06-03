/*
    control.h
    
    Manages vehicle control
    WIP
    
    @authors Orfeas Magoulas
*/

#ifndef CONTROL_H
#define CONTROL_H

#include <vehicle.h>
#include <nav.h>
#include <guidance.h>

class Control
{
private:
    Vehicle* vehicle;
    Nav* nav;
public:
    Control(Vehicle* vehicle, Nav* nav, Guidance* guidance) : vehicle(vehicle), nav(nav)  {}
    void init();
    void run();
};

inline void Control::init()
{
    
}

inline void Control::run()
{
    
}

#endif