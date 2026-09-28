#include "LightSphere.h"

real LightSphere::checkSphereIntersection(Ray& ray, const Vec3r& center) {
    Vec3r oc = ray.start_point - center;

    real a = dotVec3r(ray.direction, ray.direction);
    real b = 2.0f * dotVec3r(oc, ray.direction);
    real c = dotVec3r(oc, oc) - radius * radius;

    real discriminant = b * b - 4 * a * c;
    
    if (discriminant < 0) {
        return std::numeric_limits<real>::infinity();
    }
    else {
        real sqrt_d = sqrt(discriminant);
        real t1 = (-b - sqrt_d) / (2.0f * a);
        real t2 = (-b + sqrt_d) / (2.0f * a);

        if (t1 > 0 && t2 > 0) {
            return std::min(t1, t2); 
        } else if (t1 > 0) {
            return t1; 
        } else if (t2 > 0) {
            return t2; 
        } else {
            return std::numeric_limits<real>::infinity();
        }
    }
}