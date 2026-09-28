#pragma once

#include "Utils.h"
#include "Scene.h"

class Shader {
public:
    Shader(Scene& scene);
    ~Shader() = default;

    Vec3r shade(const Ray& ray, const HitRecord& hit_record);

private:
    Vec3r computeDiffuse(const HitRecord& hit_record, const PointLight& light);
    Vec3r computeSpecular(const HitRecord& hit_record, const PointLight& light, const Ray& ray);

    bool inShadow(const HitRecord& hit_record, const PointLight& light);

    real fresnelConductor(const Ray& ray, const HitRecord& hit_record);
    real fresnelDielectric(const Ray& ray, const HitRecord& hit_record, real n1, real n2);

    Scene& scene;
};