#include "Utils.h"
#include "Texture.h"
#include "Sphere.h"
#include <random>
#include <algorithm>



Vec3r computeOrthoBasis(Vec3r norm){
    Vec3r u;
    if (std::abs(norm.x) <= std::abs(norm.y) && std::abs(norm.x) <= std::abs(norm.z)) {
        u.x = 0;
        u.y = -norm.z;
        u.z = norm.y;
    }
    else if (std::abs(norm.y) <= std::abs(norm.x) && std::abs(norm.y) <= std::abs(norm.z)) {
        u.x = -norm.z;
        u.y = 0;
        u.z = norm.x;
    }
    else {
        u.x = -norm.y;
        u.y = norm.x;
        u.z = 0;
    }

    u = normalizeVec3r(u);
    return u;
}

float toRadians(float degrees) {
    return degrees * (3.14159 / 180.0f);
}

float getRandomFloat() {
    static thread_local std::mt19937 generator(std::random_device{}());
    std::uniform_real_distribution<float> distribution(0.0f, 1.0f);
    return distribution(generator);
}

Vec3r applyBackgroundTexture(int curr_w, int curr_h, int img_width, int img_height, const TextureMap& texMap, const Image& texImg) {
    Vec3r color(0.0);

    real tex_w, tex_h;
    tex_w = (real)(curr_w * texImg.width) / img_width;
    tex_h = (real)(curr_h * texImg.height) / img_height;

    if (texMap.interpolation == "bilinear") {
        color = applyBilinearInterpolation(texImg.data, texImg.width, texImg.height, texImg.channels, tex_w, tex_h);
    }
    if (texMap.interpolation == "nearest") {
        color = applyNearestInterpolation(texImg.data, texImg.width, texImg.height, texImg.channels, tex_w, tex_h);
    }
    printVec3r(color);
    return color * 255;
}

Vec3r applyBilinearInterpolation(float* image, int width, int height, int offset, real tex_w, real tex_h) {
    Vec3r color(0.0);
    
    int floor_w = floor(tex_w);
    int floor_h = floor(tex_h);

    real dx = tex_w - floor_w;
    real dy = tex_h - floor_h;

    color = fetchImage(image, width, height, offset, floor_w, floor_h) * (1-dx) * (1-dy) +
            fetchImage(image, width, height, offset, floor_w + 1, floor_h) * (dx) * (1-dy) +
            fetchImage(image, width, height, offset, floor_w, floor_h + 1) * (1-dx) * (dy) +
            fetchImage(image, width, height, offset, floor_w + 1, floor_h + 1) * (dx) * (dy);
    return color;
}

Vec3r applyNearestInterpolation(float* image, int width, int height, int offset, real tex_w, real tex_h) {
    Vec3r color;

    int w, h;
    int ceil_w = ceil(tex_w);
    int floor_w = floor(tex_w);
    int ceil_h = ceil(tex_h);
    int floor_h = floor(tex_h);
    
    if (ceil_w - tex_w < tex_w - floor_w) w = ceil_w;
    else w = floor_w;
    if (ceil_h - tex_h < tex_h - floor_h) h = ceil_h;
    else h = floor_h;

    color = fetchImage(image, width, height, offset, w, h);

    return color;
}

Vec3r fetchImage(float* image, int width, int height, int offset, int w, int h) {
    Vec3r color;

    if (w < 0) w = 0;
    if (w >= width) w = width - 1;
    if (h < 0) h = 0;
    if (h >= height) h = height - 1;
    
    int index = (h * width + w) * offset;

    color.x = image[index];
    color.y = image[index + 1];
    color.z = image[index + 2];
    
    color.x = pow(color.x, 1.0f / 2.2f);
    color.y = pow(color.y, 1.0f / 2.2f);
    color.z = pow(color.z, 1.0f / 2.2f);

    return color;
}

Vec3r sampleTextureColor(const TextureMap& tex_map, const Image& tex_img, real tex_u, real tex_v) {
    Vec3r replace_kd(0.0);

    real tex_w = tex_u * tex_img.width;
    real tex_h = tex_v * tex_img.height;

    if (tex_map.interpolation == "bilinear" || tex_map.interpolation == "") {
        replace_kd = applyBilinearInterpolation(tex_img.data, tex_img.width, tex_img.height, tex_img.channels, tex_w, tex_h);
    }
    else if (tex_map.interpolation == "nearest") {
        replace_kd = applyNearestInterpolation(tex_img.data, tex_img.width, tex_img.height, tex_img.channels, tex_w, tex_h);
    }
    
    return replace_kd;
}

Vec3r clampVec3r(const Vec3r& color, real min, real max) {
    Vec3r clampedVec3r;
    clampedVec3r.x = std::max(min, std::min(max, color.x));
    clampedVec3r.y = std::max(min, std::min(max, color.y));
    clampedVec3r.z = std::max(min, std::min(max, color.z));
    return clampedVec3r;
}

Vec3r normalizeVec3r(const Vec3r& v) {
    real magnitude = sqrt(dotVec3r(v, v));
    if (magnitude > 1e-9) {
        return v / magnitude;
    } else {
        return v;
    }
}

Vec3r crossVec3r(const Vec3r& v1, const Vec3r& v2) {
    Vec3r result;

    result.x = v1.y * v2.z - v1.z * v2.y;
    result.y = v1.z * v2.x - v1.x * v2.z; 
    result.z = v1.x * v2.y - v1.y * v2.x; 

    return result;
}

real euclideanDistanceVec3r(const Vec3r& v1, const Vec3r& v2) {
    real dx = v1.x - v2.x;
    real dy = v1.y - v2.y;
    real dz = v1.z - v2.z;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

real lengthSquared(const Vec3r& v) {
    return sqrt(dotVec3r(v, v));
}

real detMat3r(const Mat3r& m) {
    real p1 = m.c1.x * (m.c2.y * m.c3.z - m.c3.y * m.c2.z);
    real p2 = m.c1.y * (m.c3.x * m.c2.z - m.c2.x * m.c3.z);
    real p3 = m.c1.z * (m.c2.x * m.c3.y - m.c2.y * m.c3.x);

    return (p1 + p2 + p3);
}

bool isZeroVec3r(const Vec3r& v) {
    return (std::abs(v.x) < 1e-9) && (std::abs(v.y) < 1e-9) && (std::abs(v.z) < 1e-9);
}

void printVec3r(const Vec3r& v) {
    std::cout << "x: " << v.x << " y: " << v.y << " z: " << v.z << std::endl;
}

bool rayAABB(Ray& r, Vec3r& minB, Vec3r& maxB, real tMax) 
{
    Vec3r invDir = {1.0 / r.direction.x, 1.0 / r.direction.y, 1.0 / r.direction.z}; 

    real tmin = (minB.x - r.start_point.x) * invDir.x;
    real tmax = (maxB.x - r.start_point.x) * invDir.x;
    if (tmin > tmax) std::swap(tmin, tmax);

    real tymin = (minB.y - r.start_point.y) * invDir.y;
    real tymax = (maxB.y - r.start_point.y) * invDir.y;
    if (tymin > tymax) std::swap(tymin, tymax);

    if (tmin > tymax || tymin > tmax)
        return false;

    if (tymin > tmin) tmin = tymin;
    if (tymax < tmax) tmax = tymax;

    real tzmin = (minB.z - r.start_point.z) * invDir.z;
    real tzmax = (maxB.z - r.start_point.z) * invDir.z;
    if (tzmin > tzmax) std::swap(tzmin, tzmax);

    if (tmin > tzmax || tzmin > tmax)
        return false;

    if (tzmin > tmin) tmin = tzmin;
    if (tzmax < tmax) tmax = tzmax;

    return tmin < tMax && tmax > 0.0;
}

void computeSphereAABB(Vec3r center, real radius, Vec3r& aabb_max, Vec3r& aabb_min) {
    aabb_max = center + radius;
    aabb_min = center - radius;
}  


void computeTriangleAABB(Vec3r v0, Vec3r v1, Vec3r v2, Vec3r& aabb_max, Vec3r& aabb_min) {
    aabb_min.x = std::min({v0.x, v1.x, v2.x});
    aabb_max.x = std::max({v0.x, v1.x, v2.x});

    aabb_min.y = std::min({v0.y, v1.y, v2.y});
    aabb_max.y = std::max({v0.y, v1.y, v2.y});

    aabb_min.z = std::min({v0.z, v1.z, v2.z});
    aabb_max.z = std::max({v0.z, v1.z, v2.z});
}

bool updateAABBs(Vec3r& main_aabb_max, Vec3r& main_aabb_min, Vec3r& aabb_max, Vec3r& aabb_min) {
    bool flag = false;
    if (main_aabb_max.x < aabb_max.x) {
        main_aabb_max.x = aabb_max.x;
        flag = true;
    }
    if (main_aabb_max.y < aabb_max.y) {
        main_aabb_max.y = aabb_max.y;
        flag = true;
    }
    if (main_aabb_max.z < aabb_max.z) {
        main_aabb_max.z = aabb_max.z;
        flag = true;
    }
    if (main_aabb_min.x > aabb_min.x) {
        main_aabb_min.x = aabb_min.x;
        flag = true;
    }
    if (main_aabb_min.y > aabb_min.y) {
        main_aabb_min.y = aabb_min.y;
        flag = true;
    }
    if (main_aabb_min.z > aabb_min.z) {
        main_aabb_min.z = aabb_min.z;
        flag = true;
    }
    return flag;
}