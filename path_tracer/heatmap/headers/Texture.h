#pragma once 

#include "Real.h"
#include <iostream>


struct Image {
    std::string path_name = "";
    size_t ID = -1;
    float* data = nullptr;
    // float* hdr_data = nullptr;
    int width = -1;
    int height = -1;
    int channels = -1;
};

struct TextureMap {
    size_t ID = -1;
    std::string type = "";
    size_t image_ID = -1;
    std::string decal_mode = "";
    std::string interpolation = "";
    std::string noise_conversion = "";
    real noise_scale = 1;
    int num_octaves = 1;
    real bump_factor = 1;
    real normalizer = -1;
    Vec3r black_color = {0,0,0};
    Vec3r white_color = {0,0,0};
    real scale = 1;
    real offset = 0;
};

class Texture {
public:
    Texture() = default;
    ~Texture() = default;

    void addImage(const Image& img) { images.push_back(img); }
    void addTexMap(const TextureMap& tex_map) { texture_maps.push_back(tex_map); }

    const Image& getImage(size_t id) const { return images[id]; }
    const TextureMap& getTexMap(size_t id) { return texture_maps[id]; }

    void setBackgroundTextureID(size_t id) { background_texture_id = id; }
    const bool isBackgroundTexture() const { return background_texture_id != -1; }
    const size_t getBackgroundTextureID() const { return background_texture_id; }

private:
    std::vector<Image> images;
    std::vector<TextureMap> texture_maps;

    size_t background_texture_id = -1;
};