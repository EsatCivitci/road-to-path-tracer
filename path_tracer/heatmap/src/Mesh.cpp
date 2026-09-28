#include "Mesh.h"
#include "BVH.h" 
#include "Texture.h"

// Constructor
Mesh::Mesh() : bvh(std::make_unique<BVH>()) {
    // Correctly initializes bvh
}

// Destructor
Mesh::~Mesh() {
    // Correctly defined here (can be empty)
}

// --- ADD THESE TWO MISSING LINES ---
Mesh::Mesh(Mesh&&) noexcept = default;
Mesh& Mesh::operator=(Mesh&&) noexcept = default;
// --- END ADDITION ---


// Your existing build function
void Mesh::buildBVH(Scene& scene) {
    bvh->setScene(&scene);
    bvh->build(*this);
}

Vec3r Mesh::computeReplaceComponent(TextureMap& tex_map, Image& tex_img, real tex_u, real tex_v) {
    Vec3r replace_kd(0.0);

    real tex_w = tex_u * tex_img.width;
    real tex_h = tex_v * tex_img.height;

    if (tex_map.interpolation == "bilinear"  || tex_map.interpolation == "") {
        replace_kd = applyBilinearInterpolation(tex_img.data, tex_img.width, tex_img.height, tex_img.channels, tex_w, tex_h);
    }
    else if (tex_map.interpolation == "nearest") {
        replace_kd = applyNearestInterpolation(tex_img.data, tex_img.width, tex_img.height, tex_img.channels, tex_w, tex_h);
    }
    
    return replace_kd;
}