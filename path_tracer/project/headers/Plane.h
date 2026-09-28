#pragma once

#include "Real.h"
#include "Utils.h"

class Plane {
public:
    Plane() = default;
    ~Plane() = default;

    real checkPlaneIntersection(Ray& ray, Vec3r a);

    // ---- ID ----
    int getID() const { return id; }
    void setID(int value) { id = value; }

    // ---- Point ID ----
    int getPointID() const { return point_id; }
    void setPointID(int value) { point_id = value; }

    // ---- Material ID ----
    int getMaterialID() const { return material_id; }
    void setMaterialID(int value) { material_id = value; }

    // ---- Normal Vector ----
    const Vec3r& getNormal() const { return normal; }
    void setNormal(const Vec3r& value) { normal = value; }

    std::vector<std::string>& getTransformIds() { return transformIds; }

    void setTransformation(const Mat4r& mat) {
        transformation = mat;
        inv_transformation = mat.inverse();
    }

    const Mat4r& getTransformation() const {
        return transformation;
    }

    const Mat4r& getInverseTransformation() const {
        return inv_transformation;
    }

    const std::vector<int>& getTextures() const { return textures; }
    bool isTextured() { return textures.size() > 0; }
    void addTextures(int id) { textures.push_back(id); }

private:
    int id;
    int point_id;
    int material_id;
    Vec3r normal;

    std::vector<std::string> transformIds;
    Mat4r transformation;
    Mat4r inv_transformation;
    std::vector<int> textures;
};
