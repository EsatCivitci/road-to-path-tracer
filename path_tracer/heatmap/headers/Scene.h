#pragma once

#include "Real.h"

#include <vector>
#include <string>
#include <unordered_map>

#include "Camera.h"
#include "PointLight.h"
#include "Material.h"
#include "Plane.h"
#include "Sphere.h"
#include "Triangle.h"
#include "Mesh.h"
#include "Transformation.h"
#include "MeshInstance.h"
#include "AreaLight.h"
#include "Texture.h"
#include "DirectionalLight.h"
#include "SpotLight.h"
#include "EnvironmentLight.h"
#include "LightMesh.h"
#include "LightSphere.h"
#include "UniformGrid.h"
#include "kdTree.h"

struct BRDF {
    std::string type = "default";
    size_t ID = -1;
    bool normalized = false;
    bool kd_fresnel = false;
    real exponent = 0;
    real refractive_index = 1.0;
};

struct Scene {
    int max_recursion_depth = 0;
    Vec3r background_color;
    real shadow_ray_epsilon = 1e-3;
    real intersection_test_epsilon = 1e-9;
    real n_air = 1;
    Texture texture;

    UniformGrid grid;
    KDTree kd_tree;

    std::vector<Camera> cameras;
    Vec3r ambient_light;
    std::vector<PointLight> point_lights;
    std::vector<AreaLight> area_lights;
    std::vector<DirectionalLight> dir_lights;
    std::vector<SpotLight> spot_lights;
    std::vector<EnvironmentLight> env_lights;

    std::vector<BRDF> BRDFs;
    std::vector<Material> materials;
    std::vector<Vec3r> vertex_data;
    std::vector<Vec2r> tex_coord_data;
    std::vector<Plane> planes;
    std::vector<Triangle> triangles;
    std::vector<Mesh> meshes;
    std::vector<Mesh> light_meshes2;
    std::vector<LightMesh> light_meshes;
    std::vector<LightSphere> light_spheres;
    std::vector<Sphere> spheres;
    std::vector<random9r> samples;
    std::vector<real> gaussian_weights;

    std::unordered_map<std::string, Transformation> transformations;
    std::vector<MeshInstance> mesh_instances;
};




