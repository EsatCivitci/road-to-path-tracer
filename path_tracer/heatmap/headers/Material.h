#pragma once

#include "Real.h"
#include <iostream>

class Material {
public:
    Material() = default;
    ~Material() = default;

    bool isMirror();
    bool isConductor();
    bool isDielectric();

    // ---- ID ----
    int getID() const { return ID; }
    void setID(int id) { ID = id; }

    // ---- Type ----
    const std::string& getType() const { return type; }
    void setType(const std::string& t) { type = t; }

    // ---- Ambient Reflectance ----
    const Vec3r& getAmbientReflectance() const { return ambient_reflectance; }
    void setAmbientReflectance(const Vec3r& v) { ambient_reflectance = v; }

    // ---- Diffuse Reflectance ----
    const Vec3r& getDiffuseReflectance() const { return diffuse_reflectance; }
    void setDiffuseReflectance(const Vec3r& v) { diffuse_reflectance = v; }

    // ---- Specular Reflectance ----
    const Vec3r& getSpecularReflectance() const { return specular_reflectance; }
    void setSpecularReflectance(const Vec3r& v) { specular_reflectance = v; }

    // ---- Phong Exponent ----
    real getPhongExponent() const { return phong_exponent; }
    void setPhongExponent(real exp) { phong_exponent = exp; }

    // ---- Mirror Reflectance ----
    const Vec3r& getMirrorReflectance() const { return mirror_reflectance; }
    void setMirrorReflectance(const Vec3r& v) { mirror_reflectance = v; }

    // ---- Refraction Index ----
    real getRefractionIndex() const { return refraction_index; }
    void setRefractionIndex(real r) { refraction_index = r; }

    // ---- Absorption Index ----
    real getAbsorptionIndex() const { return absorption_index; }
    void setAbsorptionIndex(real a) { absorption_index = a; }

    // ---- Absorption Coefficient ----
    const Vec3r& getAbsorptionCoefficient() const { return absorption_coefficient; }
    void setAbsorptionCoefficient(const Vec3r& v) { absorption_coefficient = v; }

    const real getRoughness() const { return roughness; }
    void setRoughness(real i_roughness) { roughness = i_roughness; }

    int getBRDFID() const { return BRDF_ID; }
    void setBRDFID(int i_BRDF_ID) { BRDF_ID = i_BRDF_ID; }

private:
    int ID;
    std::string type = "default";
    Vec3r ambient_reflectance = {0, 0, 0};
    Vec3r diffuse_reflectance = {0, 0, 0};
    Vec3r specular_reflectance = {0, 0, 0};
    real phong_exponent = 1;
    Vec3r mirror_reflectance = {0, 0, 0};
    real refraction_index = -1;
    real absorption_index = -1;
    Vec3r absorption_coefficient = {0, 0, 0};
    real roughness = -1;
    int BRDF_ID = -1;
};