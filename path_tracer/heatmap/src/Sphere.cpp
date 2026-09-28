#include "Sphere.h"
#include "Texture.h"

real Sphere::checkSphereIntersection(Ray& ray, const Vec3r& center) {
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

Vec3r Sphere::computeReplaceComponent(TextureMap& tex_map, Image& tex_img, real tex_u, real tex_v) {
    Vec3r replace_kd(0.0);

    real tex_w = tex_u * tex_img.width;
    real tex_h = tex_v * tex_img.height;

    if (tex_map.interpolation == "bilinear" || tex_map.interpolation == "") {
        replace_kd = applyBilinearInterpolation(tex_img.data, tex_img.width, tex_img.height, tex_img.channels, tex_w, tex_h);
    }
    else if (tex_map.interpolation == "nearest") {
        replace_kd = applyNearestInterpolation(tex_img.data, tex_img.width, tex_img.height, tex_img.channels, tex_w, tex_h);
    }
    
    return replace_kd;
}

Vec3r Sphere::computeTangentU(Vec3r local_p, real pi) {
    Vec3r tangent_u;
    tangent_u.x = 2 * pi * local_p.z;
    tangent_u.y = 0;
    tangent_u.z = 2 * pi * local_p.x;
    return normalizeVec3r(tangent_u);
}


Vec3r Sphere::computeTangentV(Vec3r local_p, real pi, real phi, real theta) {
    Vec3r tangent_v;
    tangent_v.x = -pi * local_p.y * cos(phi);
    tangent_v.y = -pi * radius * sin(theta);
    tangent_v.z = -pi * local_p.y * sin(phi);
    return normalizeVec3r(tangent_v);
}

Vec3r Sphere::computeNormalMapNormals(Vec3r T, Vec3r B, Vec3r N, Vec3r MN) {
    Vec3r new_normal;
    new_normal.x = T.x * MN.x + B.x * MN.y + N.x * MN.z;
    new_normal.y = T.y * MN.x + B.y * MN.y + N.y * MN.z;
    new_normal.z = T.z * MN.x + B.z * MN.y + N.z * MN.z;
    return normalizeVec3r(new_normal);
}