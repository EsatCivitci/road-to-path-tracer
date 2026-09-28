#include "Material.h"

bool Material::isMirror() {
    return type == "mirror";
}

bool Material::isConductor() {
    return type == "conductor";
}

bool Material::isDielectric() {
    return type == "dielectric";
}
