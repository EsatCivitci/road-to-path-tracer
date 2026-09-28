#pragma once

#include "Real.h"

class AreaLight {
public:
    AreaLight() = default;
    ~AreaLight() = default;

    Vec3r getLightDirection(const Vec3r& point, const Vec3r& point_sample) const;
    real getLightDistance(const Vec3r& direction) const;

    int getID() const { return ID; }
    const Vec3r& getPosition() const { return position; }
    const Vec3r& getNormal() const { return normal; }
    const real getSize() const { return size; }
    const Vec3r& getRadiance() const { return radiance; }
    const Vec3r& getU() const { return u; }
    const Vec3r& getV() const { return v; }

    void setID(int i_ID) { ID = i_ID; }
    void setPosition(Vec3r i_position) { position = i_position; }
    void setNormal(Vec3r i_normal) { normal = i_normal; }
    void setSize(real i_size) { size = i_size; }
    void setRadiance(Vec3r i_radiance) { radiance = i_radiance; }
    void setU(Vec3r i_u) { u = i_u; }
    void setv(Vec3r i_v) { v = i_v; }

    real nSamples;

private:
    int ID = -1;
    Vec3r position = {0,0,0};
    Vec3r normal = {0,0,0};
    real size = -1;
    Vec3r radiance = {0,0,0};
    Vec3r u = {0,0,0};
    Vec3r v = {0,0,0};
};