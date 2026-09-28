#include "ToneMap.h"
#include "Utils.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

void ToneMap::applyToneMapping(const std::vector<real>& data, std::string name, int nx, int ny) {
    size_t size_data = data.size();
    size_t size_lum = size_data / 3;

    std::vector<real> L_w;
    computeL_w(data, L_w);

    real L_w_prime = computeL_w_prime(L_w);

    std::vector<unsigned char> image_data;
    for (auto& tm : tm_list) {
        if (tm.TMO == "Photographic") {
            std::vector<real> L;
            computeL(tm.key, L_w_prime, L_w, L);

            real L_white = computeL_white(L, tm.burn_out);

            for (int i = 0; i < size_lum; ++i)  {
                real L_d = (L[i] * (1 + L[i] / pow(L_white, 2))) / (1 + L[i]); 
                // real L_d = L[i]  /  (1 + L[i]); 

                Vec3r color = operateColor(L_d, L_w[i], tm.saturation, tm.gamma, data[3*i], data[3*i+1], data[3*i+2]);

                unsigned char R = static_cast<unsigned char>(color.x);
                unsigned char G = static_cast<unsigned char>(color.y);
                unsigned char B = static_cast<unsigned char>(color.z);

                image_data.push_back(R);
                image_data.push_back(G);
                image_data.push_back(B);
            }

            int stride_bytes = nx * 3; 
            std::string output_path = "my_outputs/" + name + tm.extension;
            stbi_write_png(output_path.c_str(), nx, ny, 3, image_data.data(), stride_bytes);
        }

        else if (tm.TMO == "ACES") {
            std::vector<real> L;
            computeL(tm.key, L_w_prime, L_w, L);

            real L_white = computeL_white(L, tm.burn_out);

            for (int i = 0; i < size_lum; ++i)  {
                real L_d = mapACES(L[i]) / mapACES(L_white); 

                Vec3r color = operateColor(L_d, L_w[i], tm.saturation, tm.gamma, data[3*i], data[3*i+1], data[3*i+2]);

                unsigned char R = static_cast<unsigned char>(color.x);
                unsigned char G = static_cast<unsigned char>(color.y);
                unsigned char B = static_cast<unsigned char>(color.z);

                image_data.push_back(R);
                image_data.push_back(G);
                image_data.push_back(B);
            }

            int stride_bytes = nx * 3; 
            std::string output_path = "my_outputs/" + name + tm.extension;
            stbi_write_png(output_path.c_str(), nx, ny, 3, image_data.data(), stride_bytes);
        }

        else if (tm.TMO == "Filmic") {
            std::vector<real> L;
            computeL(tm.key, L_w_prime, L_w, L);

            real L_white = computeL_white(L, tm.burn_out);

            for (int i = 0; i < size_lum; ++i)  {
                real L_d = mapFilmic(L[i]) / mapFilmic(L_white); 

                Vec3r color = operateColor(L_d, L_w[i], tm.saturation, tm.gamma, data[3*i], data[3*i+1], data[3*i+2]);

                unsigned char R = static_cast<unsigned char>(color.x);
                unsigned char G = static_cast<unsigned char>(color.y);
                unsigned char B = static_cast<unsigned char>(color.z);

                image_data.push_back(R);
                image_data.push_back(G);
                image_data.push_back(B);
            }

            int stride_bytes = nx * 3; 
            std::string output_path = "my_outputs/" + name + tm.extension;
            stbi_write_png(output_path.c_str(), nx, ny, 3, image_data.data(), stride_bytes);
        }

        image_data.clear();
    }
}

real ToneMap::mapACES(real L) {
    return ((L * (L * A + B)) / (L * (L * C + D) + E));
}

real ToneMap::mapFilmic(real L) {
    return ((L * (L * a + c * b) + d * e) / (L * (a * L + b) + d * f)) - e / f;
}

Vec3r ToneMap::operateColor(real L_d, real L_w, real saturation, real gamma, real R, real G, real B) {
    Vec3r color;
    real eps = 1e-6;

    color.x = pow(R / (L_w + eps), saturation) * L_d;
    color.y = pow(G / (L_w + eps), saturation) * L_d;
    color.z = pow(B / (L_w + eps), saturation) * L_d;

    clampVec3r(color, 0.0, 1.0);

    color.x = pow(color.x, 1.0f / gamma);
    color.y = pow(color.y, 1.0f / gamma);
    color.z = pow(color.z, 1.0f / gamma);

    color = clampVec3r(color * 255, 0, 255);

    return color;
}

void ToneMap::computeL(real key, real L_w_prime,const std::vector<real>& L_w, std::vector<real>& L) {
    size_t size = L_w.size();
    real key_over_L_w_prime = key / L_w_prime;
    for (int i = 0; i < size; ++i) {
        L.push_back(key_over_L_w_prime * L_w[i]);
    }
}

real ToneMap::computeL_white(const std::vector<real>& L, real burn_out) {
    size_t size = L.size();
    std::vector<real> L_sorted = L; 
    std::sort(L_sorted.begin(), L_sorted.end());
    real percentile = (100 - burn_out) / 100;
    if (percentile >= 0.99) {
        percentile = 0.99;
    }   
    real L_white = L_sorted[floor(size * percentile)];

    return L_white;
}

real ToneMap::computeL_w_prime(const std::vector<real>& L_w) {
    size_t size = L_w.size();
    real epsilon = 1e-4;
    real L_w_prime = 0;
    for (int i = 0; i < size; ++i) {
        L_w_prime += log(epsilon + L_w[i]);
    }
    L_w_prime = exp(L_w_prime / size);
    return L_w_prime;
}

void ToneMap::computeL_w(const std::vector<real> data, std::vector<real>& L_w) {
    size_t size = data.size();
    for (int i = 0; i < size; i+=3) {
        real R = data[i];
        real G = data[i+1];
        real B = data[i+2];

        real Y = R * 0.2126 + G * 0.7152 + B * 0.0722;
        L_w.push_back(Y);
    }
}





