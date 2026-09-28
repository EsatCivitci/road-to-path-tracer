#pragma once

#include "Scene.h"
#include "Utils.h"
#include "Shader.h"
#include "Perlin.h"

class RayTracer {
public:
    RayTracer(Scene& input_scene); 
    ~RayTracer();

    void startRayTracer();

private:

    Vec3r computeColor(Ray& ray);
    Vec3r computeEnvironmentColor(Image tex_img, Vec3r d, std::string type);
    Vec3r computeHeatMapColor(int cost);

    bool closestHit(Ray& ray, HitRecord& hit_record);    
    bool closestHitGrid(Ray& ray, HitRecord& hit_record);
    bool closestHitKDTree(Ray& ray, HitRecord& hit_record);

    bool updateHitRecord(HitRecord& rec, real t, const Vec3r& point, const Vec3r& normal, int matID);
    bool processPlaneHit(Ray& ray, Plane& plane, HitRecord& hit_record);
    bool processSphereHit(Ray& ray, Sphere& sphere, HitRecord& hit_record);
    bool processTriangleHit(Ray& ray, Triangle& triangle, HitRecord& hit_record);
    bool processMeshHit(Ray& ray, Mesh& mesh, HitRecord& hit_record);
    bool processMeshInstanceHit(Ray& ray, MeshInstance& inst, HitRecord& hit_record);
    bool processLightSphereHit(Ray& ray, LightSphere& lSphere, HitRecord& hit_record);
    bool processLightMeshHit(Ray& ray, LightMesh& lMesh, HitRecord& hit_record);
    bool processGridMeshHit(Ray& ray, Mesh& mesh, int triangle_id, HitRecord& hit_record);
    bool processGridMeshInstanceHit(Ray& ray, MeshInstance& inst, Mesh& baseMesh, int triangle_id, HitRecord& hit_record);
    bool intersectBLAS(Ray& localRay, Mesh& mesh, HitRecord& rec);

    Vec3r applyShading(Ray& ray, const HitRecord& hit_record);
    void applyTextureShading (HitRecord& hit_record, const std::vector<int>& textureIDs, const TextureContext& ctx, const Mat4r& invTranspose); 
    Vec3r pathTracing(Ray& ray, const HitRecord& hit_record);

    Vec3r computeBRDF(BRDF brdf, Vec3r norm, Vec3r wi, Vec3r wo, Material mat, HitRecord hit_record);

    Vec3r diffuseTerm(const HitRecord& hit_record, const PointLight& light);
    Vec3r specularTerm(const HitRecord& hit_record, const PointLight& light, Ray& ray);

    Vec3r diffuseTermArea(const HitRecord& hit_record, const AreaLight& light, const Vec3r& point);
    Vec3r specularTermArea(const HitRecord& hit_record, const AreaLight& light, Ray& ray, const Vec3r& point);

    Vec3r diffuseTermDirection(const HitRecord& hit_record, const DirectionalLight& light);
    Vec3r specularTermDirection(const HitRecord& hit_record, const DirectionalLight& light, Ray& ray);

    Vec3r diffuseTermSpot(const HitRecord& hit_record, const SpotLight& light, Vec3r light_direction, real angle);
    Vec3r specularTermSpot(const HitRecord& hit_record, const SpotLight& light, Ray& ray, Vec3r light_direction, real angle);

    bool inShadow(const HitRecord& hit_record, const PointLight& light, Vec3r light_direction, real light_distance);
    bool inShadowGrid(const HitRecord& hit_record, const PointLight& light, Vec3r light_direction, real light_distance);
    bool anyHitGrid(Ray& shadow_ray, real max_distance);

    Ray reflect(Ray& ray, const HitRecord& hit_record);
    Ray refract(Ray& ray, const HitRecord& hit_record, real n1, real n2);

    real computeFresnelConductor(Ray& ray, const HitRecord& hit_record);
    real computeFresnelDielectric(Ray& ray, const HitRecord& hit_record, real n1, real n2);

    Ray transformRay(const Ray& ray, const Mat4r& mat);
    void transformHitToWorld(const Mat4r& objToWorld,
                                        const Vec3r& localHit,
                                        const Vec3r& localNormal,
                                        Vec3r& worldHit,
                                        Vec3r& worldNormal);
    
    std::vector<real> precomputeGaussianWeights(int nSamples);
    void generateRandoms(real grid_dim);

    Scene& scene; 
    int curr_sample_index = -1;

    Perlin per;
    std::string renderer = "ApplyShading";
};