#include "Perlin.h"
#include "Utils.h"


real Perlin::noise(Vec3r point, std::string conversion) {
    int i = floor(point.x);
    int j = floor(point.y);
    int k = floor(point.z);

    real c = 0.0;

    for (int a = 0; a < 8; ++a) {
        Vec3i ijk = {i, j, k};
        ijk = ijk + addTable[a];

        int idx;
        idx = table[abs(ijk.z) % 16];
        idx = table[abs(ijk.y + idx) % 16];
        idx = table[abs(ijk.x + idx) % 16];
        Vec3r g = gradients[idx];

        real dx = point.x - ijk.x;
        real dy = point.y - ijk.y;
        real dz = point.z - ijk.z;
        Vec3r d = {dx, dy, dz};

        c += f(dx) * f(dy) * f(dz) * dotVec3r(g, d);
    }

    if (conversion == "absval") {
        c = abs(c);
    }
    else {
        c = (c + 1) / 2;
    }
    if (c < 0.001) c = 0.0;

    return c;
}

real Perlin::f(real x) {
    real result = 0;
    if (abs(x) < 1) {
        result = -6 * pow(abs(x), 5) + 
                 15 * pow(abs(x), 4) +
                -10 * pow(abs(x), 3) + 1;
    }
    return result;
}