#pragma once

#include "Real.h"
#include "Triangle.h"

class LightMesh {
public:
    // ---- ID ----
    int getID() const { return id; }
    void setID(int value) { id = value; }

    // ---- Material ID ----
    int getMaterialID() const { return material_id; }
    void setMaterialID(int value) { material_id = value; }

    // ---- Triangles ----
    std::vector<Triangle>& getTriangles() { return triangles; }
    void setTriangles(const std::vector<Triangle>& value) { triangles = value; }
    void addTriangle(const Triangle& tri) { triangles.push_back(tri); }

    // ---- Radiance ----
    Vec3r getRadiance() const {return radiance;}
    void setRadiance(Vec3r i_radiance) {radiance = i_radiance;}

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

private:
    int id;
    int material_id;
    Vec3r radiance;
    std::vector<Triangle> triangles;
    std::vector<std::string> transformIds;
    Mat4r transformation;
    Mat4r inv_transformation;
};