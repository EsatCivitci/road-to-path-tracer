#pragma once

#include "Real.h"

class PointLight {
public: 
    PointLight() = default;
    ~PointLight() = default;

    Vec3r getLightDirection(const Vec3r& point) const;
    real getLightDistance(const Vec3r& direction) const;

    // Getters!!
    int getID() const { return ID; }
    const Vec3r& getPosition() const { return position; }
    const Vec3r& getIntensity() const { return intensity; }

    // Setters!!
    void setID(int i_ID) { ID = i_ID; }
    void setPosition(const Vec3r& i_position) { position = i_position; }
    void setIntensity(const Vec3r& i_intensity) { intensity = i_intensity; }

    std::vector<std::string>& getTransformIds() { return transformIds; }

private:
    int ID = -1;
    Vec3r position = {0,0,0};
    Vec3r intensity = {0,0,0};
    std::vector<std::string> transformIds;

};