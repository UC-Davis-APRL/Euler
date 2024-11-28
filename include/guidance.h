/*
    guidance.h

    Manages vehicle guidance
    WIP

    @authors Orfeas Magoulas
*/

#ifndef GUIDANCE_H
#define GUIDANCE_H

#include <nav.h>

class Guidance
{
private:
    Nav *nav;

public:
    Guidance(Nav *nav) : nav(nav) {}
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