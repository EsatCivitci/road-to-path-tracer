#include "PointLight.h"
#include "Utils.h"

Vec3r PointLight::getLightDirection(const Vec3r& point) const {
    return position - point;
}

real PointLight::getLightDistance(const Vec3r& direction) const {
    return sqrt(dotVec3r(direction, direction));
}