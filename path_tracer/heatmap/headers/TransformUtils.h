#pragma once

#include "Utils.h"
#include <iostream>
#include <iomanip>
#include <string>

inline void printMat4r(const Mat4r& mat, const std::string& label = "") {
    if (!label.empty())
        std::cout << "=== " << label << " ===\n";

    std::cout << std::fixed << std::setprecision(5);
    for (int i = 0; i < 4; ++i) {
        std::cout << "  ";
        for (int j = 0; j < 4; ++j) {
            std::cout << std::setw(10) << mat.m[i][j] << " ";
        }
        std::cout << "\n";
    }
    std::cout << "-----------------------------\n";
}

inline void printVec3r(const Vec3r& v, const std::string& label = "") {
    if (!label.empty()) std::cout << label << ": ";
    std::cout << "x: " << v.x << " y: " << v.y << " z: " << v.z << std::endl;
}

// Applies a 4x4 transformation matrix to a 3D point (w = 1)
inline Vec3r applyTransformToPoint(const Mat4r& mat, const Vec3r& point) {
    glm::vec4 p(point.x, point.y, point.z, 1.0);
    p = mat.toGlm() * p;
    return Vec3r(p.x, p.y, p.z);
}

// Applies a 4x4 transformation matrix to a 3D direction vector (w = 0)
inline Vec3r applyTransformToVector(const Mat4r& mat, const Vec3r& vec) {
    glm::vec4 v(vec.x, vec.y, vec.z, 0.0);
    v = mat.toGlm() * v;
    glm::vec3 normalized = glm::normalize(glm::vec3(v));
    return Vec3r(normalized.x, normalized.y, normalized.z);
}

// Applies a 4x4 transformation matrix to a 3D direction vector (w = 0)
inline Vec3r applyTransformToVectorSpecial(const Mat4r& mat, const Vec3r& vec) {
    glm::vec4 v(vec.x, vec.y, vec.z, 0.0);
    v = mat.toGlm() * v;
    return Vec3r(v.x, v.y, v.z);
}

// --- Compose a single transformation matrix from a list of IDs ---
inline Mat4r composeTransformFromIds(
    const std::vector<std::string>& transformIds,
    const std::unordered_map<std::string, Transformation>& allTransforms,
    const std::string& ownerName // for logging
) {
    Mat4r total = Mat4r::identity();

    for (const std::string& tid : transformIds) {
        auto it = allTransforms.find(tid);
        if (it != allTransforms.end()) {
            total = it->second.getMatrix() * total;
        } else {
            std::cerr << "[Warning] " << ownerName
                      << " refers to missing transformation ID '" << tid << "'\n";
        }
    }

    return total;
}

inline void computeWorldAABB(
    const Vec3r& localMin,
    const Vec3r& localMax,
    const Mat4r& transform,
    Vec3r& outMin,
    Vec3r& outMax
) {
    std::vector<Vec3r> corners = {
        {localMin.x, localMin.y, localMin.z},
        {localMin.x, localMin.y, localMax.z},
        {localMin.x, localMax.y, localMin.z},
        {localMin.x, localMax.y, localMax.z},
        {localMax.x, localMin.y, localMin.z},
        {localMax.x, localMin.y, localMax.z},
        {localMax.x, localMax.y, localMin.z},
        {localMax.x, localMax.y, localMax.z}
    };

    Vec3r newMin(std::numeric_limits<real>::infinity());
    Vec3r newMax(-std::numeric_limits<real>::infinity());

    for (const auto& c : corners) {
        Vec3r wc = applyTransformToPoint(transform, c);
        newMin.x = std::min(newMin.x, wc.x);
        newMin.y = std::min(newMin.y, wc.y);
        newMin.z = std::min(newMin.z, wc.z);
        newMax.x = std::max(newMax.x, wc.x);
        newMax.y = std::max(newMax.y, wc.y);
        newMax.z = std::max(newMax.z, wc.z);
    }

    outMin = newMin;
    outMax = newMax;
}

inline int getMeshByID(const Scene& scene, int id) {
    for (size_t i = 0; i < scene.meshes.size(); ++i) {
        if (scene.meshes[i].getID() == id)
            return static_cast<int>(i);
    }
    return -1;
}

inline int getMeshInstanceByID(const Scene& scene, int id) {
    for (size_t i = 0; i < scene.mesh_instances.size(); ++i) {
        if (scene.mesh_instances[i].id == id)
            return static_cast<int>(i);
    }
    return -1;
}

inline Mat4r computeInstanceTransformRecursive(
    const MeshInstance& instance,
    const Scene& scene)
{
    Mat4r local = instance.transformation;

    if (instance.baseMeshId <= 0)
        return local;

    int baseIdx = instance.baseMeshId;

    int meshID = getMeshByID(scene, baseIdx);
    // Base is a mesh
    if (meshID != -1) {
        const Mesh& baseMesh = scene.meshes[meshID];
        return instance.resetTransform ? local : local * baseMesh.getTransformation();
    }
    int instID = getMeshInstanceByID(scene, baseIdx);
    if (instID != -1) {
        const MeshInstance& baseInst = scene.mesh_instances[instID];
        // This approach is wrong!!!
        // baseInst's transformation maybe not set yet
        // Fix this when you have time!!!
        Mat4r baseTransform = baseInst.transformation;
        return instance.resetTransform ? local : local * baseTransform;
    }

    return local;
}
