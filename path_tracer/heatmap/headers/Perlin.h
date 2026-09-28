#pragma once
#include "Real.h"

#include <iostream>
#include <algorithm>
#include "cmath" 

class Perlin {
public:
    Perlin() = default;
    ~Perlin() = default;

    real noise(Vec3r point, std::string conversion);

private:
    real f(real x); 
    int table[16] = { 
        13, 2, 8, 15, 0, 9, 5, 1, 
        14, 4, 11, 7, 10, 6, 12, 3 
    };

    Vec3r gradients[16] = {
        Vec3r(1,1,0),
        Vec3r(-1,1,0),
        Vec3r(1,-1,0),
        Vec3r(-1,-1,0),
        Vec3r(1,0,1),
        Vec3r(-1,0,1),
        Vec3r(1,0,-1),
        Vec3r(-1,0,-1),
        Vec3r(0,1,1),
        Vec3r(0,-1,1),
        Vec3r(0,1,-1),
        Vec3r(0,-1,-1),
        Vec3r(1,1,0),
        Vec3r(-1,1,0),
        Vec3r(0,-1,1),
        Vec3r(0,-1,-1)
    };

    Vec3i addTable[8] = {
        Vec3i(0,0,0),
        Vec3i(1,0,0),
        Vec3i(0,1,0),
        Vec3i(0,0,1),
        Vec3i(1,1,0),
        Vec3i(1,0,1),
        Vec3i(0,1,1),
        Vec3i(1,1,1),
    };
};