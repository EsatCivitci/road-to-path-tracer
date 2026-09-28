#include "kdTree.h"
#include "Scene.h"
#include "TransformUtils.h"

#include <algorithm>

inline real surfaceArea(const Vec3r& min, const Vec3r& max) {
    Vec3r d = max - min;
    return 2.0f * (d.x * d.y + d.y * d.z + d.z * d.x);
}

void KDTree::build(Scene& scene, SplitMethod method) {

    nodes.clear();
    nodes.reserve(2048); 

    std::vector<BuildPrimitive> all_primitives;
    
    auto addPrim = [&](int id, TYPE type, const Vec3r& min, const Vec3r& max, int meshID = -1) {
        BuildPrimitive bp;
        bp.ref.id = id;        
        bp.ref.type = type;
        bp.ref.mesh_id = meshID; 
        bp.min = min;
        bp.max = max;
        bp.center = (min + max) * 0.5f; 
        all_primitives.push_back(bp);
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
        addPrim(sphere.getID(), TYPE::Sphere, min, max);
    }

    for (const auto& lSphere : scene.light_spheres) {
        Vec3r min, max;
        Vec3r localCenter = scene.vertex_data[lSphere.getCenterVertexID() - 1];
        computeTransformedSphereAABB(localCenter, lSphere.getRadius(), lSphere.getTransformation(), min, max);
        addPrim(lSphere.getID(), TYPE::LightSphere, min, max);
    }

    for (const auto& tri : scene.triangles) {
        Vec3r min, max;
        Vec3r v0 = scene.vertex_data[tri.getV0ID() - 1];
        Vec3r v1 = scene.vertex_data[tri.getV1ID() - 1];
        Vec3r v2 = scene.vertex_data[tri.getV2ID() - 1];
        computeTriangleAABB(v0, v1, v2, max, min);
        addPrim(tri.getID(), TYPE::Triangle, min, max);
    }

    auto processMeshes = [&](auto& meshList, TYPE type) {
        for (auto& mesh : meshList) {
            int mesh_id = mesh.getID();
            Mat4r transform = mesh.getTransformation(); 
            std::vector<Triangle>& tris = mesh.getTriangles(); 
            
            int local_tri_idx = 0; 

            for (auto& tri : tris) {
                Vec3r min, max;
                Vec3r v0 = scene.vertex_data[tri.getV0ID() - 1];
                Vec3r v1 = scene.vertex_data[tri.getV1ID() - 1];
                Vec3r v2 = scene.vertex_data[tri.getV2ID() - 1];

                v0 = applyTransformToPoint(transform, v0);
                v1 = applyTransformToPoint(transform, v1);
                v2 = applyTransformToPoint(transform, v2);

                computeTriangleAABB(v0, v1, v2, max, min);
                addPrim(local_tri_idx, type, min, max, mesh_id);
                
                local_tri_idx++;
            }
        }
    };

    processMeshes(scene.meshes, TYPE::Mesh);
    processMeshes(scene.light_meshes, TYPE::LightMesh);

    for (const auto& mesh_inst : scene.mesh_instances) {
        int mesh_id = mesh_inst.id;
        Mat4r transform = mesh_inst.transformation;

        int baseIdx = mesh_inst.baseMeshObjectId;
        int id = getMeshByID(scene, baseIdx);
        Mesh& baseMesh = scene.meshes[id];

        int local_tri_idx = 0;

        for (auto& tri : baseMesh.getTriangles()) {
            Vec3r v0 = scene.vertex_data[tri.getV0ID() - 1];
            Vec3r v1 = scene.vertex_data[tri.getV1ID() - 1];
            Vec3r v2 = scene.vertex_data[tri.getV2ID() - 1];

            v0 = applyTransformToPoint(transform, v0);
            v1 = applyTransformToPoint(transform, v1);
            v2 = applyTransformToPoint(transform, v2);

            Vec3r min, max;
            computeTriangleAABB(v0, v1, v2, max, min);
            
            addPrim(local_tri_idx, TYPE::MeshInstance, min, max, mesh_id);
            local_tri_idx++;
        }
    }

    Vec3r root_min(1e30f, 1e30f, 1e30f);
    Vec3r root_max(-1e30f, -1e30f, -1e30f);
    
    if (all_primitives.empty()) {
        root_min = Vec3r(0,0,0);
        root_max = Vec3r(0,0,0);
    } 
    else {
        for (auto& p : all_primitives) {
            updateAABBs(root_max, root_min, p.max, p.min);
        }
    }

    this->root_min = root_min;
    this->root_max = root_max;

    buildRecursive(all_primitives, root_min, root_max, 0, method);

    int a = 5;
    int b = 5;
    int c = 5;
    int d = 5;
}

const real K_TRAVERSAL = 1.0f;
const real K_INTERSECT = 8.0f;

int KDTree::buildRecursive(std::vector<BuildPrimitive>& objects, 
                           const Vec3r& node_min, const Vec3r& node_max, 
                           int depth, SplitMethod method) {
    int node_idx = nodes.size();
    nodes.emplace_back();

    if (objects.size() <= 4 || depth > 25) {
        nodes[node_idx].split_axis = -1; 
        for (const auto& bp : objects) {
            nodes[node_idx].primitives.push_back(bp.ref);
        }
        return node_idx;
    }

    int best_axis = -1;
    real best_pos = 0;

    real root_area = surfaceArea(node_min, node_max);
    real leaf_cost = K_INTERSECT * objects.size() * root_area;

    real best_cost = leaf_cost;

    if (method == SplitMethod::SAH) {
        for (int axis = 0; axis < 3; ++axis) {
            if (node_max[axis] - node_min[axis] < 1e-4f) continue;

            const int BINS = 8; 
            int bin_counts[BINS] = {0};

            real axis_width = node_max[axis] - node_min[axis];
            real scale = BINS / axis_width;

            for (auto& obj : objects) {
                int binIdx = (int)((obj.center[axis] - node_min[axis]) * scale);
                if (binIdx < 0) binIdx = 0;
                if (binIdx >= BINS) binIdx = BINS - 1;

                bin_counts[binIdx]++;
            }

            int countL = 0;
            
            for (int i = 0; i < BINS - 1; ++i) {
                countL += bin_counts[i];
                int countR = objects.size() - countL;
                
                if (countL == 0 || countR == 0) continue;

                real current_split_pos = node_min[axis] + (i + 1) * axis_width / BINS;

                Vec3r left_dim = node_max - node_min;
                left_dim[axis] = current_split_pos - node_min[axis];
                real areaL = 2.0f * (left_dim.x * left_dim.y + left_dim.y * left_dim.z + left_dim.z * left_dim.x);

                Vec3r right_dim = node_max - node_min;
                right_dim[axis] = node_max[axis] - current_split_pos;
                real areaR = 2.0f * (right_dim.x * right_dim.y + right_dim.y * right_dim.z + right_dim.z * right_dim.x);

                real split_cost = K_TRAVERSAL * root_area + K_INTERSECT * (areaL * countL + areaR * countR);

                if (split_cost < best_cost) {
                    best_cost = split_cost;
                    best_axis = axis;
                    best_pos = current_split_pos;
                }
            }
        }
    }
    else {
        Vec3r extent = node_max - node_min;
        best_axis = 0;
        if (extent.y > extent.x) best_axis = 1;
        if (extent.z > extent[best_axis]) best_axis = 2;

        if (method == SplitMethod::SpatialMedian) {
            best_pos = (node_min[best_axis] + node_max[best_axis]) * 0.5f;
        } 
        else if (method == SplitMethod::ObjectMedian) {
            best_pos = findObjectMedian(objects, best_axis);
        }

        best_cost = -1.0f; 
    }

    if (best_cost >= leaf_cost || best_axis == -1) {
        nodes[node_idx].split_axis = -1;
        for (const auto& bp : objects) nodes[node_idx].primitives.push_back(bp.ref);
        return node_idx;
    }

    std::vector<BuildPrimitive> left_objs;
    std::vector<BuildPrimitive> right_objs;
    
    left_objs.reserve(objects.size()); 
    right_objs.reserve(objects.size());

    for (const auto& obj : objects) {
        if (obj.min[best_axis] <= best_pos) left_objs.push_back(obj);
        if (obj.max[best_axis] >= best_pos) right_objs.push_back(obj);
    }
    if (left_objs.size() == objects.size() && right_objs.size() == objects.size()) {
         nodes[node_idx].split_axis = -1;
         for (const auto& bp : objects) nodes[node_idx].primitives.push_back(bp.ref);
         return node_idx;
    }

    nodes[node_idx].split_axis = best_axis;
    nodes[node_idx].split_pos = best_pos;

    Vec3r left_max = node_max; left_max[best_axis] = best_pos;
    Vec3r right_min = node_min; right_min[best_axis] = best_pos;

    nodes[node_idx].left_child = buildRecursive(left_objs, node_min, left_max, depth + 1, method);
    nodes[node_idx].right_child = buildRecursive(right_objs, right_min, node_max, depth + 1, method);

    return node_idx;
}

real KDTree::findObjectMedian(std::vector<BuildPrimitive>& objects, int axis) {
    if (objects.empty()) return 0;

    size_t mid = objects.size() / 2;
    std::nth_element(objects.begin(), objects.begin() + mid, objects.end(),
        [axis](const BuildPrimitive& a, const BuildPrimitive& b) {
            return a.center[axis] < b.center[axis];
        });
    return objects[mid].center[axis];
}

bool KDTree::intersectAABB(const Ray& ray, real& t_enter, real& t_exit) {
    real tmin = 0.0f; 
    real tmax = std::numeric_limits<real>::infinity();
    const real EPSILON = 1e-6;

    for (int i = 0; i < 3; ++i) {
        if (std::abs(ray.direction[i]) < EPSILON) {
            if (ray.start_point[i] < root_min[i] || ray.start_point[i] > root_max[i]) return false;
            continue;
        }

        real invD = 1.0f / ray.direction[i];
        real t0 = (root_min[i] - ray.start_point[i]) * invD;
        real t1 = (root_max[i] - ray.start_point[i]) * invD;

        if (t0 > t1) std::swap(t0, t1);

        tmin = (t0 > tmin) ? t0 : tmin;
        tmax = (t1 < tmax) ? t1 : tmax;

        if (tmax < tmin) return false;
    }

    t_enter = tmin;
    t_exit = tmax;
    return true;
}

