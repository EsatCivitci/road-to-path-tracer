#include "Shader.h"

Shader::Shader(Scene& scene) : scene(scene) 
{
    
}


Vec3r Shader::computeDiffuse(const HitRecord& hit_record, const PointLight& light) {
    Material material = scene.materials[hit_record.material_id - 1];

    Vec3r light_direction = light.getLightDirection(hit_record.intersection_point);
    real light_distance = light.getLightDistance(light_direction);
    Vec3r wi = normalizeVec3r(light_direction);

    Vec3r irradiance = light.getIntensity() / pow(light_distance,2);
    real cos_theta = std::max((real)0, dotVec3r(wi, hit_record.normal));

    return material.getDiffuseReflectance() * cos_theta * irradiance;
}

Vec3r Shader::computeSpecular(const HitRecord& hit_record, const PointLight& light, const Ray& ray) {
    Material material = scene.materials[hit_record.material_id - 1];

    Vec3r light_direction = light.getLightDirection(hit_record.intersection_point);
    real light_distance = light.getLightDistance(light_direction);
    Vec3r wi = normalizeVec3r(light_direction);

    Vec3r irradiance = light.getIntensity() / pow(light_distance,2);
    Vec3r wo = normalizeVec3r(-ray.direction);

    Vec3r h = normalizeVec3r(wi + wo);
    real cos_alpha = std::max((real)0, dotVec3r(hit_record.normal, h));

    return material.getSpecularReflectance() * pow(cos_alpha, material.getPhongExponent()) * irradiance;
}

bool Shader::inShadow(const HitRecord& hit_record, const PointLight& light) {
    Vec3r light_direction = light.getLightDirection(hit_record.intersection_point);
    Vec3r wi = normalizeVec3r(light_direction);

    real light_distance = sqrt(dotVec3r(light_direction, light_direction));

    Ray shadow_ray;
    shadow_ray.start_point = hit_record.intersection_point +  hit_record.normal * scene.shadow_ray_epsilon;
    shadow_ray.direction = wi;

    for (auto& plane : scene.planes) {
        Vec3r a = scene.vertex_data[plane.getPointID() - 1];
        real t = plane.checkPlaneIntersection(shadow_ray, a);
        if (t < light_distance) {
            return true;
        }
    }

    for (auto& sphere : scene.spheres) {
        Vec3r center = scene.vertex_data[sphere.getCenterVertexID() - 1];
        real t = sphere.checkSphereIntersection(shadow_ray, center);
        if (t < light_distance) {
            return true;
        }
    }

    for (auto& triangle : scene.triangles) {
        Vec3r a = scene.vertex_data[triangle.getV0ID()-1];
        Vec3r b = scene.vertex_data[triangle.getV1ID()-1];
        Vec3r c = scene.vertex_data[triangle.getV2ID()-1];
        real t = triangle.checkTriangleIntersection(shadow_ray, a, b, c, "flat");
        if (t < light_distance) {
            return true;
        }
    }

    for (auto& mesh : scene.meshes) {
        std::string shading_mode = mesh.getShadingMode();
        std::vector<Triangle> triangles = mesh.getTriangles();
        for (auto& triangle : triangles) {
            Vec3r a = scene.vertex_data[triangle.getV0ID()-1];
            Vec3r b = scene.vertex_data[triangle.getV1ID()-1];
            Vec3r c = scene.vertex_data[triangle.getV2ID()-1];
            real t = triangle.checkTriangleIntersection(shadow_ray, a, b, c, shading_mode);
            if (t < light_distance) {
                return true;
            }
        }
    }
    return false;
}

real Shader::fresnelDielectric(const Ray& ray, const HitRecord& hit_record, real n1, real n2) {
    real cos_theta = dotVec3r(-ray.direction, hit_record.normal);
    if (cos_theta < 0) {
        cos_theta = -cos_theta; 
    }

    real eta = n1 / n2;
    real discriminant = 1 - pow(eta, 2) * (1 - pow(cos_theta, 2));
    real cos_phi;
    if (discriminant > 0) {
        real cos_phi = sqrt(discriminant);

        real r_parallel = (n2 * cos_theta - n1 * cos_phi) / (n2 * cos_theta + n1 * cos_phi);
        real r_perpendicular = (n1 * cos_theta - n2 * cos_phi) / (n1 * cos_theta + n2 * cos_phi);
        return 0.5f * (pow(r_parallel, 2) + pow(r_perpendicular, 2));
    }
    return (real)1;
}

real Shader::fresnelConductor(const Ray& ray, const HitRecord& hit_record) {
    Material mat = scene.materials[hit_record.material_id - 1];
    real cos_theta = std::max((real)0, dotVec3r(-ray.direction, hit_record.normal));
    real n1 = ray.n;
    real n2 = mat.getRefractionIndex();
    real k2 = mat.getAbsorptionIndex();

    real a = pow(n2, 2) + pow(k2, 2);
    real b = 2 * n2 * cos_theta;
    real c = pow(cos_theta, 2);

    real Rs = (a - b + c) / (a + b + c);
    real Rp = (a * c - b + 1) / (a * c + b + 1);

    return 0.5 * (Rs + Rp);
}


