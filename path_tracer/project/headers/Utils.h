#pragma once

#include "Real.h"
#include <iostream>
#include <cfloat>
#include <cmath>
#include <algorithm>
#include <vector>
#include <iomanip>

class Triangle;
class Sphere;
class Texture;

struct TextureMap;
struct Image;


struct Ray {
    Vec3r start_point;
    Vec3r direction;
    real distance = 0;
    int depth;
    bool isIn = false;
    real n = (real)1;
    real beta = -1;
    real gamma = -1;
    int tri_idx = -1;
    std::string renderer = "";
};

struct Mat3r{
    Vec3r c1;
    Vec3r c2;
    Vec3r c3;
};

struct HitRecord {
    real t = std::numeric_limits<real>::infinity(); 
    int material_id = -1;                          
    Vec3r intersection_point;   
    Vec3r normal; 
    Vec3r kd = {0, 0, 0};
    Vec3r ks = {0, 0, 0};
    Vec3r object_radiance = {0, 0, 0};
    bool replaceAll = false;
    bool isCheckerboard = false;
    bool lightObject = false;
    real area = 0;
    real total_area = 0;
};

struct TextureContext {
    Vec3r localPoint;   
    Vec3r localNormal;  
    Vec3r tangentU;   
    Vec3r tangentV;     
    real u;            
    real v;             
};

enum class TYPE {
    Triangle,
    Sphere,
    Mesh,
    MeshInstance,
    LightSphere,
    LightMesh
};

template <typename T>
T clamp(T val, T low, T high) {
    return std::max(low, std::min(val, high));
}

Vec3r computeOrthoBasis(Vec3r norm);

float getRandomFloat();
float toRadians(float degrees);

void computeSphereAABB(Vec3r center, real radius, Vec3r& aabb_max, Vec3r& aabb_min);
void computeTriangleAABB(Vec3r v0, Vec3r v1, Vec3r v2, Vec3r& aabb_max, Vec3r& aabb_min);
bool updateAABBs(Vec3r& main_aabb_max, Vec3r& main_aabb_min, Vec3r& aabb_max, Vec3r& aabb_min);

Vec3r applyBackgroundTexture(int curr_w, int curr_h, int img_width, int img_height, const TextureMap& texMap, const Image& texImg);
Vec3r applyBilinearInterpolation(float* image, int width, int height, int offset, real tex_w, real tex_h);
Vec3r applyNearestInterpolation(float* image, int width, int height, int offset,real tex_w, real tex_h);
Vec3r fetchImage(float* image, int width, int height, int offset,int w, int h);
Vec3r sampleTextureColor(const TextureMap& tex_map, const Image& tex_img, real tex_u, real tex_v);

Vec3r clampVec3r(const Vec3r& color, real min, real max);
Vec3r normalizeVec3r(const Vec3r& v);
Vec3r crossVec3r(const Vec3r& v1, const Vec3r& v2);
real euclideanDistanceVec3r(const Vec3r& v1, const Vec3r& v2);
real lengthSquared(const Vec3r& v);
real detMat3r(const Mat3r& m);
bool isZeroVec3r(const Vec3r& v);
void printVec3r(const Vec3r& v);

bool rayAABB(Ray& r, Vec3r& minB, Vec3r& maxB, real tMax);

inline Vec2r operator-(const Vec2r& v1, const Vec2r& v2) {
    return {v1.x - v2.x, v1.y - v2.y};
}

inline real dotVec3r(const Vec3r& v1, const Vec3r& v2) {
    return (v1.x * v2.x + v1.y * v2.y + v1.z * v2.z);
}

inline Vec3i operator+(const Vec3i& v1, const Vec3i& v2) {
    return {v1.x + v2.x, v1.y + v2.y, v1.z + v2.z};
}

inline Vec3r operator-(const Vec3r& v1) {
    return {-v1.x, -v1.y,-v1.z};
}

inline Vec3r operator+(const Vec3r& v1, const Vec3r& v2) {
    return {v1.x + v2.x, v1.y + v2.y, v1.z + v2.z};
}

inline Vec3r operator+(const Vec3r& v1, const real& s) {
    return {v1.x + s, v1.y + s, v1.z + s};
}

inline Vec3r operator-(const Vec3r& v1, const Vec3r& v2) {
    return {v1.x - v2.x, v1.y - v2.y, v1.z - v2.z};
}

inline Vec3r operator*(const Vec3r& v1, const Vec3r& v2) {
    return {v1.x * v2.x, v1.y * v2.y, v1.z * v2.z};
}

inline Vec3r operator*(const Vec3r& v, real s) {
    return {v.x * s, v.y * s, v.z * s};
}

inline Vec3r operator*(real s, const Vec3r& v) {
    return v * s; 
}

inline Vec3r operator/(const Vec3r& v, real s) {
    return {v.x / s, v.y / s, v.z / s};
}

inline Vec3r operator/(real s, const Vec3r& v) {
    return {s / v.x, s / v.y, s / v.z};
}

inline Vec3r applyPointTransform(const Vec3r& p, const Mat4r& M) {
    glm::mat<4,4,real,glm::defaultp> gm;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            gm[c][r] = M.m[r][c];  // convert to column-major

    glm::vec4 res = gm * glm::vec4(p.x, p.y, p.z, 1.0);
    return Vec3r(res.x, res.y, res.z);
}

inline Vec3r applyVectorTransform(const Vec3r& v, const Mat4r& M) {
    glm::mat<4,4,real,glm::defaultp> gm;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            gm[c][r] = M.m[r][c];

    glm::vec4 res = gm * glm::vec4(v.x, v.y, v.z, 0.0);
    return normalizeVec3r(Vec3r(res.x, res.y, res.z));
}


