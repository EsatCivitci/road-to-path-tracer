#pragma once

#include "Real.h"

class DirectionalLight {
public:
    DirectionalLight() = default;
    ~DirectionalLight() = default;

    int getID() const { return ID; }
    const Vec3r& getDirection() const { return direction; }
    const Vec3r& getRadiance() const { return radiance; }

    void setID(int i_ID) { ID = i_ID; }
    void setDirection(Vec3r i_direction) { direction = i_direction; }
    void setRadiance(Vec3r i_radiance) { radiance = i_radiance; }
    
private:
    int ID = -1;
    Vec3r direction = {0,0,0};
    Vec3r radiance = {0,0,0};

};