#include "Triangle.h"

real Triangle::checkTriangleIntersection(Ray& ray, Vec3r a, Vec3r b, Vec3r c, std::string shading_mode) {
    if (!ray.isIn && dotVec3r(ray.direction, normal) > 0) return std::numeric_limits<real>::infinity();

    Vec3r ao = a - ray.start_point;

    Mat3r A;
    A.c1 = a-b;
    A.c2 = a-c;
    A.c3 = ray.direction;
    real detA = detMat3r(A);

    if (std::abs(detA) < intersection_test_epsilon) {
        return std::numeric_limits<real>::infinity();
    }

    Mat3r mat_beta = A;
    mat_beta.c1 = ao;
    real det_mat_beta = detMat3r(mat_beta);
    real beta = det_mat_beta / detA;

    if (beta < 0) {
        return std::numeric_limits<real>::infinity();
    }

    Mat3r mat_gama = A;
    mat_gama.c2 = ao;
    real det_mat_gama = detMat3r(mat_gama);
    real gama = det_mat_gama / detA;

    if (gama < 0) {
        return std::numeric_limits<real>::infinity();
    }

    if (gama + beta > 1) {
        return std::numeric_limits<real>::infinity();
    }

    if (shading_mode == "smooth") {
        real alpha = 1 - gama - beta;
        setShadingNormal(normalizeVec3r(alpha * v0_normal + beta * v1_normal + gama * v2_normal));
    }

    ray.beta = beta;
    ray.gamma = gama;


    Mat3r mat_t = A;
    mat_t.c3 = ao;
    real det_mat_t = detMat3r(mat_t);
    real t = det_mat_t / detA;

    if (t > 0) {
        return t;
    }
    else {
        return std::numeric_limits<real>::infinity();
    }
}

Vec3r Triangle::computeTangentVectorU(Vec2r texCoord_v0, Vec2r texCoord_v1, Vec2r texCoord_v2, Vec3r a, Vec3r b, Vec3r c) {
    Vec3r deltaPos1 = b - a;
    Vec3r deltaPos2 = c - a;

    Vec2r deltaUV1 = texCoord_v1 - texCoord_v0;
    Vec2r deltaUV2 = texCoord_v2 - texCoord_v0;

    real r = 1.0f / (deltaUV1.x * deltaUV2.y - deltaUV1.y * deltaUV2.x);
    Vec3r tangent = (deltaPos1 * deltaUV2.y - deltaPos2 * deltaUV1.y) * r;

    return normalizeVec3r(tangent);
}

Vec3r Triangle::computeTangentVectorV(Vec2r texCoord_v0, Vec2r texCoord_v1, Vec2r texCoord_v2, Vec3r a, Vec3r b, Vec3r c) {
    Vec3r deltaPos1 = b - a;
    Vec3r deltaPos2 = c - a;

    Vec2r deltaUV1 = texCoord_v1 - texCoord_v0;
    Vec2r deltaUV2 = texCoord_v2 - texCoord_v0;

    real r = 1.0f / (deltaUV1.x * deltaUV2.y - deltaUV1.y * deltaUV2.x);
    Vec3r tangent = (deltaPos2 * deltaUV1.x - deltaPos1 * deltaUV2.x) * r;

    return normalizeVec3r(tangent);
}

Vec3r Triangle::computeNormalMapNormals(Vec3r T, Vec3r B, Vec3r N, Vec3r MN) {
    Vec3r new_normal;
    new_normal.x = T.x * MN.x + B.x * MN.y + N.x * MN.z;
    new_normal.y = T.y * MN.x + B.y * MN.y + N.y * MN.z;
    new_normal.z = T.z * MN.x + B.z * MN.y + N.z * MN.z;
    return normalizeVec3r(new_normal);
}


