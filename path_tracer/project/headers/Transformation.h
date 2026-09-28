#pragma once
#include "Real.h"      // for Mat4r, real
#include <string>

class Transformation {
public:
    enum class Type {
        None,
        Translation,
        Scaling,
        Rotation,
        Composite
    };

    Transformation() : type(Type::None), mat(Mat4r::identity()) {}

    // ---- Constructors for each transformation ----
    static Transformation Translation(real dx, real dy, real dz);
    static Transformation Scaling(real sx, real sy, real sz);
    static Transformation Rotation(real angleDegrees, real axisX, real axisY, real axisZ);
    static Transformation Composite(const real values[16]); 

    // ---- Get the 4x4 matrix result ----
    const Mat4r& getMatrix() const { return mat; }

    // Optional: return type as string (useful for debug)
    std::string typeName() const;

private:
    Transformation(Type t, const Mat4r& m) : type(t), mat(m) {}

    Type type;
    Mat4r mat;
};