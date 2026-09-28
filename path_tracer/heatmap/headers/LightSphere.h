#pragma once

#include "Real.h"
#include "Utils.h"

class LightSphere {
public:
    // ---- ID ----
    int getID() const { return id; }
    void setID(int value) { id = value; }

    // ---- Material ID ----
    int getMaterialID() const { return material_id; }
    void setMaterialID(int value) { material_id = value; }

    // ---- Center Vertex ID ----
    int getCenterVertexID() const { return center_vertex_id; }
    void setCenterVertexID(int value) { center_vertex_id = value; }

    // ---- Radius ----
    real getRadius() const { return radius; }
    void setRadius(real value) { radius = value; }

    // ---- Radiance ----
    Vec3r getRadiance() const {return radiance;}
    void setRadiance(Vec3r i_radiance) {radiance = i_radiance;}

    std::vector<std::string>& getTransformIds() { return transformIds; }

    real checkSphereIntersection(Ray& ray, const Vec3r& center);

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
    int center_vertex_id;
    real radius;
    Vec3r radiance;
    std::vector<std::string> transformIds;
    Mat4r transformation;
    Mat4r inv_transformation;
};