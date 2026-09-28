#include "Plane.h"

real Plane::checkPlaneIntersection(Ray& ray, Vec3r a) {

    real d_n = dotVec3r(ray.direction, normal);
    if (d_n >= 0) {
        return std::numeric_limits<real>::infinity();
    }

    Vec3r ao = a - ray.start_point;
    real t = dotVec3r(ao, normal) / d_n;

    if (t > 1e-6) {
        return t;
    }
    else {
        return std::numeric_limits<real>::infinity();
    }
}