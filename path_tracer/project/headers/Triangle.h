#pragma once 

#include "Utils.h"

class Triangle {
public:
    Triangle() = default;
    ~Triangle() = default;

    real checkTriangleIntersection(Ray& ray, Vec3r a, Vec3r b, Vec3r c, std::string shading_mode);

    // ---- ID ----
    int getID() const { return id; }
    void setID(int value) { id = value; }

    // ---- Material ID ----
    int getMaterialID() const { return material_id; }
    void setMaterialID(int value) { material_id = value; }

    // ---- Vertex Indices ----
    int getV0ID() const { return v0_id; }
    void setV0ID(int value) { v0_id = value; }

    int getV1ID() const { return v1_id; }
    void setV1ID(int value) { v1_id = value; }

    int getV2ID() const { return v2_id; }
    void setV2ID(int value) { v2_id = value; }

    void setVertexIDs(int v0, int v1, int v2) {
        v0_id = v0;
        v1_id = v1;
        v2_id = v2;
    }

    // ---- Texture Indices ----
    int getT0ID() const { return t0_id; }
    void setT0ID(int value) { t0_id = value; }

    int getT1ID() const { return t1_id; }
    void setT1ID(int value) { t1_id = value; }

    int getT2ID() const { return t2_id; }
    void setT2ID(int value) { t2_id = value; }

    void setTextureIDs(int t0, int t1, int t2) {
        t0_id = t0;
        t1_id = t1;
        t2_id = t2;
    }

    // ---- Vertex Normals (per corner) ----
    const Vec3r& getV0Normal() const { return v0_normal; }
    void setV0Normal(const Vec3r& value) { v0_normal = value; }

    const Vec3r& getV1Normal() const { return v1_normal; }
    void setV1Normal(const Vec3r& value) { v1_normal = value; }

    const Vec3r& getV2Normal() const { return v2_normal; }
    void setV2Normal(const Vec3r& value) { v2_normal = value; }

    // (Optional convenience function)
    void setVertexNormals(const Vec3r& n0, const Vec3r& n1, const Vec3r& n2) {
        v0_normal = n0;
        v1_normal = n1;
        v2_normal = n2;
    }

    // ---- Normal ----
    const Vec3r& getNormal() const { return normal; }
    void setNormal(const Vec3r& value) { normal = value; }

    // ---- Shading Normal ----
    const Vec3r& getShadingNormal() const { return shading_normal; }
    void setShadingNormal(const Vec3r& value) { shading_normal = value; }

    // ---- Centroid ----
    const Vec3r& getCentroid() const { return centroid; }
    void setCentroid(const Vec3r& value) { centroid = value; }

    // ---- Area ----
    const real& getArea() const { return area; }
    void setArea(const real& i_area) { area = i_area; }

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

    Vec3r computeTangentVectorU(Vec2r texCoord_v0, Vec2r texCoord_v1, Vec2r texCoord_v2, Vec3r a, Vec3r b, Vec3r c);
    Vec3r computeTangentVectorV(Vec2r texCoord_v0, Vec2r texCoord_v1, Vec2r texCoord_v2, Vec3r a, Vec3r b, Vec3r c);

    Vec3r computeNormalMapNormals(Vec3r T, Vec3r B, Vec3r N, Vec3r MN);

private:
    int id;
    int material_id;
    int v0_id, v1_id, v2_id;
    int t0_id, t1_id, t2_id;
    Vec3r centroid;
    Vec3r v0_normal, v1_normal, v2_normal;
    Vec3r normal;
    Vec3r shading_normal;
    real intersection_test_epsilon = 1e-9;
    std::vector<std::string> transformIds;
    Mat4r transformation;
    Mat4r inv_transformation;
    real area;
};
