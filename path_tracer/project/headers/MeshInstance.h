#pragma once

#include <string>
#include <vector>
#include "Real.h"
#include "Transformation.h"

struct MeshInstance {
    MeshInstance() {}

    bool isMotionBlurred() { return (motion_vector.x != 0 || motion_vector.y != 0 || motion_vector.z != 0); }

    int id = -1;                      
    int baseMeshId = -1;               
    int materialId = -1;               
    bool resetTransform = false;   
    int baseMeshObjectId = -1;   

    std::vector<std::string> transformIds; 

    Mat4r transformation;
    Mat4r inv_transformation;
    Vec3r motion_vector = {0,0,0};

    Vec3r aabb_min_world;
    Vec3r aabb_max_world;
};