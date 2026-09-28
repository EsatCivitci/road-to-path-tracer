#pragma once

#include "Real.h"
#include <iostream>

class EnvironmentLight {
public:
    EnvironmentLight() = default;
    ~EnvironmentLight() = default;

    int getID() const { return ID; }
    std::string getType() const { return type; }
    int getImageID() const { return image_ID; }
    std::string getSampler() const { return sampler; }

    void setID(int i_ID) { ID = i_ID; }
    void setType(std::string i_type) { type = i_type; }
    void setImageID(int i_image_ID) { image_ID = i_image_ID; }
    void setSampler(std::string i_sampler) { sampler = i_sampler; }

private:
    int ID = -1;
    std::string type = "";
    int image_ID = -1;
    std::string sampler = "cosine";
};