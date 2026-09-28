#pragma once

#include "Real.h"

class SpotLight {
public:
    SpotLight() = default;
    ~SpotLight() = default;

    bool getLightDirection(const Vec3r& point, Vec3r& dir, real& angle) const;
    real getLightDistance(const Vec3r& direction) const;

    int getID() const { return ID; }
    const Vec3r& getPosition() const { return position; }
    const Vec3r& getDirection() const { return direction; }
    const Vec3r& getIntensity() const { return intensity; }
    real getCoverageAngle() const { return coverage_angle; }
    real getFallOffAngle() const { return fall_off_angle; }

    void setID(int i_ID) { ID = i_ID; }
    void setPosition(Vec3r i_position) { position = i_position; }
    void setDirection(Vec3r i_direction) { direction = i_direction; }
    void setIntensity(Vec3r i_intensity) { intensity = i_intensity; }
    void setCoverageAngle(real i_coverage_angle) { coverage_angle = i_coverage_angle; }
    void setFallOffAngle(real i_fall_off_angle) { fall_off_angle = i_fall_off_angle; }
    
private:
    int ID = -1;
    Vec3r position = {0,0,0};
    Vec3r direction = {0,0,0};
    Vec3r intensity = {0,0,0};
    real coverage_angle = 0;
    real fall_off_angle = 0;
};