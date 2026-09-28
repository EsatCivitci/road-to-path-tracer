#pragma once

#include "Triangle.h"
#include <memory>

class BVH;
struct Scene;

class Mesh {
public:
    Mesh();
    ~Mesh(); // Declared here

    // --- THESE MUST BE DECLARATIONS ONLY ---
    Mesh(Mesh&&) noexcept;
    Mesh& operator=(Mesh&&) noexcept;
    // --- END CHANGE ---

    // Delete copy operations (this is correct)
    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;

    BVH& getBVH() { return *bvh; }
    void buildBVH(Scene& scene);

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

    // ---- Shading Mode ----
    const std::string& getShadingMode() const { return shading_mode; }
    void setShadingMode(const std::string& mode) { shading_mode = mode; }

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

    Vec3r& getMotionVector() { return motion_vector; }
    void setMotionVector(const Vec3r& i_motion_vector) { motion_vector = i_motion_vector; }
    bool isMotionBlurred() { return (motion_vector.x != 0 && motion_vector.y != 0 && motion_vector.z != 0); }

    const std::vector<int>& getTextures() const { return textures; }
    bool isTextured() { return textures.size() > 0; }
    void addTextures(int id) { textures.push_back(id); }

    Vec3r computeReplaceComponent(TextureMap& tex_map, Image& tex_img, real tex_u, real tex_v);

    void setIsLight(bool i_val) { isLight = i_val; }
    bool getIsLight() { return isLight; }

private:
    int id;
    int material_id;
    std::vector<Triangle> triangles;
    std::string shading_mode = "flat";
    std::unique_ptr<BVH> bvh;
    std::vector<std::string> transformIds;
    Mat4r transformation;
    Mat4r inv_transformation;
    Vec3r motion_vector = {0,0,0};
    std::vector<int> textures;
    Vec3r radiance;
    bool isLight = false;
};
