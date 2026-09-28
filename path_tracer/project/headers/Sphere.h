#pragma once

#include "Utils.h"

class Texture;
struct TextureMap;
struct Image;

class Sphere {
public:
    Sphere() = default;
    ~Sphere() = default;

    real checkSphereIntersection(Ray& ray, const Vec3r& center);
    Vec3r computeReplaceComponent(TextureMap& tex_map, Image& tex_img, real tex_u, real tex_v);
    Vec3r computeTangentU(Vec3r local_p, real pi);
    Vec3r computeTangentV(Vec3r local_p, real pi, real phi, real theta);
    Vec3r computeNormalMapNormals(Vec3r T, Vec3r B, Vec3r N, Vec3r MN);

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
    int material_id;
    int center_vertex_id;
    real radius;
    std::vector<std::string> transformIds;
    Mat4r transformation;
    Mat4r inv_transformation;
    std::vector<int> textures;
};
