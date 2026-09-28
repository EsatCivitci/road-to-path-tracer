#pragma once

#include "Real.h"
#include "Utils.h"

#include <iostream>
#include <vector>
#include <cfloat>

struct Scene;
class Mesh;  

enum Axis { X_AXIS = 0, Y_AXIS = 1, Z_AXIS = 2 };

struct BVHNode {
    Vec3r aabb_min {  FLT_MAX,  FLT_MAX,  FLT_MAX };
    Vec3r aabb_max { -FLT_MAX, -FLT_MAX, -FLT_MAX };
    uint left_child = 0, right_child = 0;
    uint firstPrim = 0, primCount = 0;
};

class BVH {
public:

    BVH() = default;
    ~BVH() = default;

    void setScene(Scene* s) { scene = s; }

    void build(Mesh& mesh);
    uint buildRecursive(uint first, uint count);

    void splitByMedian(uint first, uint count, Axis axis);
    Axis pickSplitAxis(uint first, uint count, BVHNode& node);

    bool intersect(Ray& r, HitRecord& rec) const;
    bool intersectNode(uint nodeIndex, Ray& r, HitRecord& rec) const;
    bool rayAABB(Ray& r, Vec3r& minB, Vec3r& maxB, real tMax) const;

    const std::vector<BVHNode>& getBVHNodes() const {
        return nodes;
    }

    int getBVHNodeSize() { return nodes.size(); }

private:
    Scene* scene = nullptr;
    Mesh* currentMesh = nullptr;

    std::vector<uint> primIndices;
    std::vector<BVHNode> nodes;

    static int counter;
};