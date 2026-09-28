#include "UniformGrid.h"
#include "Scene.h"
#include "TransformUtils.h"

bool UniformGrid::buildGrid(Scene& scene) {
    if (!computeMainGrid(scene)) {
        std::cout << "Main Grid Not Initialized!!" << std::endl;
        return false;
    }
    
    size_t num_objects = main_grid.objects.size();
    if (num_objects == 0) return true;
    std::cout << "num: " << num_objects << std::endl;

    Vec3r scene_size = main_grid.aabb_max - main_grid.aabb_min;

    if (scene_size.x <= 0) scene_size.x = 0.0001;
    if (scene_size.y <= 0) scene_size.y = 0.0001;
    if (scene_size.z <= 0) scene_size.z = 0.0001;

    float lambda = 5.0f; 
    float volume = scene_size.x * scene_size.y * scene_size.z;
    
    float factor = std::pow((lambda * num_objects) / volume, 1.0f / 3.0f);

    nx = static_cast<int>(std::floor(scene_size.x * factor));
    ny = static_cast<int>(std::floor(scene_size.y * factor));
    nz = static_cast<int>(std::floor(scene_size.z * factor));

    std::cout << nx << "x" << ny << "x" << nz << std::endl;

    if (nx < 1) nx = 1;
    if (ny < 1) ny = 1;
    if (nz < 1) nz = 1;

    cell_size.x = scene_size.x / nx;
    cell_size.y = scene_size.y / ny;
    cell_size.z = scene_size.z / nz;


    grid_lst.clear();
    grid_lst.resize(nx * ny * nz);

    int grid_idx = 0;
    for (const auto& obj : main_grid.objects) {
        int i_min = static_cast<int>((obj.aabb_min.x - main_grid.aabb_min.x) / cell_size.x);
        int j_min = static_cast<int>((obj.aabb_min.y - main_grid.aabb_min.y) / cell_size.y);
        int k_min = static_cast<int>((obj.aabb_min.z - main_grid.aabb_min.z) / cell_size.z);

        int i_max = static_cast<int>((obj.aabb_max.x - main_grid.aabb_min.x) / cell_size.x);
        int j_max = static_cast<int>((obj.aabb_max.y - main_grid.aabb_min.y) / cell_size.y);
        int k_max = static_cast<int>((obj.aabb_max.z - main_grid.aabb_min.z) / cell_size.z);

        i_min = std::max(0, std::min(i_min, nx - 1));
        j_min = std::max(0, std::min(j_min, ny - 1));
        k_min = std::max(0, std::min(k_min, nz - 1));

        i_max = std::max(0, std::min(i_max, nx - 1));
        j_max = std::max(0, std::min(j_max, ny - 1));
        k_max = std::max(0, std::min(k_max, nz - 1));

        for (int k = k_min; k <= k_max; ++k) {
            for (int j = j_min; j <= j_max; ++j) {
                for (int i = i_min; i <= i_max; ++i) {
                    int cell_index = i + (j * nx) + (k * nx * ny);
                    grid_lst[cell_index].push_back(grid_idx);
                }
            }
        }
        grid_idx++;
    }

    return true; 
}   

bool UniformGrid::computeMainGrid(Scene& scene) {
    auto addObjectToGrid = [&](int id, TYPE type, const Vec3r& min, const Vec3r& max, int meshID = -1) {
        gObject grid_object;
        grid_object.id = id;
        grid_object.obj_type = type;
        grid_object.mesh_id = meshID;
        grid_object.aabb_min = min;
        grid_object.aabb_max = max;

        Vec3r temp_max = max;
        Vec3r temp_min = min;
        updateAABBs(main_grid.aabb_max, main_grid.aabb_min, temp_max, temp_min);
        
        main_grid.objects.push_back(grid_object);
    };

    auto computeTransformedSphereAABB = [&](const Vec3r& localCenter, real radius, const Mat4r& transform, Vec3r& outMin, Vec3r& outMax) {
        Vec3r minL = localCenter - Vec3r(radius);
        Vec3r maxL = localCenter + Vec3r(radius);
        
        Vec3r corners[8] = {
            Vec3r(minL.x, minL.y, minL.z), Vec3r(maxL.x, minL.y, minL.z),
            Vec3r(minL.x, maxL.y, minL.z), Vec3r(maxL.x, maxL.y, minL.z),
            Vec3r(minL.x, minL.y, maxL.z), Vec3r(maxL.x, minL.y, maxL.z),
            Vec3r(minL.x, maxL.y, maxL.z), Vec3r(maxL.x, maxL.y, maxL.z)
        };

        outMin = Vec3r(1e30f);
        outMax = Vec3r(-1e30f);

        for (int i = 0; i < 8; ++i) {
            Vec3r worldPt = applyTransformToPoint(transform, corners[i]);
            
            if (worldPt.x < outMin.x) outMin.x = worldPt.x;
            if (worldPt.y < outMin.y) outMin.y = worldPt.y;
            if (worldPt.z < outMin.z) outMin.z = worldPt.z;

            if (worldPt.x > outMax.x) outMax.x = worldPt.x;
            if (worldPt.y > outMax.y) outMax.y = worldPt.y;
            if (worldPt.z > outMax.z) outMax.z = worldPt.z;
        }
    };

    for (const auto& sphere : scene.spheres) {
        Vec3r min, max;
        Vec3r localCenter = scene.vertex_data[sphere.getCenterVertexID() - 1];
        
        computeTransformedSphereAABB(localCenter, sphere.getRadius(), sphere.getTransformation(), min, max);
        
        addObjectToGrid(sphere.getID(), TYPE::Sphere, min, max);
    }

    for (const auto& lSphere : scene.light_spheres) {
        Vec3r min, max;
        Vec3r localCenter = scene.vertex_data[lSphere.getCenterVertexID() - 1];
        
        computeTransformedSphereAABB(localCenter, lSphere.getRadius(), lSphere.getTransformation(), min, max);
        
        addObjectToGrid(lSphere.getID(), TYPE::LightSphere, min, max);
    }

    for (const auto& tri : scene.triangles) {
        Vec3r min, max;
        Vec3r v0 = scene.vertex_data[tri.getV0ID() - 1];
        Vec3r v1 = scene.vertex_data[tri.getV1ID() - 1];
        Vec3r v2 = scene.vertex_data[tri.getV2ID() - 1];
        computeTriangleAABB(v0, v1, v2, max, min);
        addObjectToGrid(tri.getID(), TYPE::Triangle, min, max);
    }

    auto processMeshes = [&](auto& meshList, TYPE type) {
        for (auto& mesh : meshList) {
            int mesh_id = mesh.getID();
            
            Mat4r transform = mesh.getTransformation(); 

            std::vector<Triangle>& tris = mesh.getTriangles(); 
            
            int i = 0;
            for (auto& tri : tris) {
                Vec3r min, max;
                Vec3r v0 = scene.vertex_data[tri.getV0ID() - 1];
                Vec3r v1 = scene.vertex_data[tri.getV1ID() - 1];
                Vec3r v2 = scene.vertex_data[tri.getV2ID() - 1];

                v0 = applyTransformToPoint(transform, v0);
                v1 = applyTransformToPoint(transform, v1);
                v2 = applyTransformToPoint(transform, v2);

                computeTriangleAABB(v0, v1, v2, max, min);
                addObjectToGrid(i++, type, min, max, mesh_id);
            }
        }
    };

    processMeshes(scene.meshes, TYPE::Mesh);
    processMeshes(scene.light_meshes, TYPE::LightMesh);

    for (const auto& mesh_inst : scene.mesh_instances) {
        int mesh_id = mesh_inst.id;
        Mat4r transform = mesh_inst.transformation;

        int meshIdx = mesh_inst.baseMeshObjectId;
        int id = getMeshByID(scene, meshIdx);
        Mesh& baseMesh = scene.meshes[id]; 

        for (auto& tri : baseMesh.getTriangles()) {
            Vec3r v0 = scene.vertex_data[tri.getV0ID() - 1];
            Vec3r v1 = scene.vertex_data[tri.getV1ID() - 1];
            Vec3r v2 = scene.vertex_data[tri.getV2ID() - 1];

            v0 = applyTransformToPoint(transform, v0);
            v1 = applyTransformToPoint(transform, v1);
            v2 = applyTransformToPoint(transform, v2);

            Vec3r min, max;
            computeTriangleAABB(v0, v1, v2, max, min);
            addObjectToGrid(tri.getID(), TYPE::MeshInstance, min, max, mesh_id);
        }
    }

    return true;
}

// Slab Method (Wikipedia definition)
/*
 * In computer graphics, the slab method is an algorithm used to 
 * solve the ray-box intersection problem in case of an axis-aligned
 * bounding box (AABB), i.e. to determine the intersection points 
 * between a ray and the box. Due to its efficient nature, that can 
 * allow for a branch-free implementation, it is widely used in computer 
 * graphics applications.
 */
bool UniformGrid::intersectAABB(Ray& ray, real& t_enter, real& t_exit) {
    real tmin = -std::numeric_limits<real>::infinity();
    real tmax = std::numeric_limits<real>::infinity();

    real epsilon = 1e-6;

    for (int i = 0; i < 3; ++i) {

        if (std::abs(ray.direction[i]) < epsilon) {
            if (ray.start_point[i] < main_grid.aabb_min[i] || 
                ray.start_point[i] > main_grid.aabb_max[i]) {
                return false;
            }
            continue;
        }

        real invD = 1.0f / ray.direction[i];
        real t0 = (main_grid.aabb_min[i] - ray.start_point[i]) * invD;
        real t1 = (main_grid.aabb_max[i] - ray.start_point[i]) * invD;

        if (t0 > t1) std::swap(t0, t1);

        tmin = (t0 > tmin) ? t0 : tmin;
        tmax = (t1 < tmax) ? t1 : tmax;

        if (tmax < tmin) {
            return false;
        }
    }

    t_enter = tmin;
    t_exit = tmax;
    return true;
}


