#include "Transformation.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

// --- Factory functions ---

Transformation Transformation::Translation(real dx, real dy, real dz) {
    return Transformation(Type::Translation, Mat4r::createTranslation(dx, dy, dz));
}

Transformation Transformation::Scaling(real sx, real sy, real sz) {
    return Transformation(Type::Scaling, Mat4r::createScaling(sx, sy, sz));
}

Transformation Transformation::Rotation(real angleDegrees, real axisX, real axisY, real axisZ) {
    real angleRadians = glm::radians(angleDegrees);
    return Transformation(Type::Rotation, Mat4r::createRotation(angleRadians, axisX, axisY, axisZ));
}

Transformation Transformation::Composite(const real values[16]) {
    return Transformation(Type::Composite, Mat4r(values));
}

std::string Transformation::typeName() const {
    switch(type) {
        case Type::Translation: return "Translation";
        case Type::Scaling:     return "Scaling";
        case Type::Rotation:    return "Rotation";
        case Type::Composite:   return "Composite";
    }
    return "Unknown";
}
