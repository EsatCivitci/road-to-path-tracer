#include "SpotLight.h"
#include "Utils.h"


bool SpotLight::getLightDirection(const Vec3r& point, Vec3r& dir, real& angle) const {
    dir = position - point;

    real cos_angle = dotVec3r(normalizeVec3r(-dir), normalizeVec3r(direction));
    angle = acos(cos_angle) * 180 / 3.141592;

    if (angle > coverage_angle / 2) return false;

    return true;
}

real SpotLight::getLightDistance(const Vec3r& direction) const {
    return sqrt(dotVec3r(direction, direction));
}
