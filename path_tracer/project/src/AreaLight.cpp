#include "AreaLight.h"
#include "Utils.h"

Vec3r AreaLight::getLightDirection(const Vec3r& point, const Vec3r& point_sample) const {
    return point_sample - point;
}

real AreaLight::getLightDistance(const Vec3r& direction) const {
    return sqrt(dotVec3r(direction, direction));
}