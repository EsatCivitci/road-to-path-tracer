#pragma once

#include <array>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp> 
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/type_ptr.hpp>

typedef double real;

struct Vec2r {
    real x,y;
};

struct Vec3r {
    real x,y,z;

    Vec3r() : x(0), y(0), z(0) {} 
    Vec3r(real xx, real yy, real zz) : x(xx), y(yy), z(zz) {}
    Vec3r(real v) : x(v), y(v), z(v) {}

    // 1. Read-only access (for const Vec3r)
    const real& operator[](int i) const {
        return (&x)[i]; 
    }

    // 2. Write access (allows vec[0] = 5.0f)
    real& operator[](int i) {
        return (&x)[i];
    }
};

struct Vec4r {
    real x, y, z, w;

    Vec4r() : x(0), y(0), z(0), w(0) {}
    Vec4r(real xx, real yy, real zz, real ww) : x(xx), y(yy), z(zz), w(ww) {}
    Vec4r(real v) : x(v), y(v), z(v), w(v) {}

    // Assumes it's a point
    Vec4r(const Vec3r& v) : x(v.x), y(v.y), z(v.z), w(1.0) {} 
};

struct random9r {
    real psi_1, psi_2, psi_3, psi_4, psi_5, psi_6, psi_7, psi_8, psi_9;
};

struct Vec3i {
    int x,y,z;

    Vec3i(int xx, int yy, int zz) : x(xx), y(yy), z(zz) {}
};

struct Vec4i {
    int x,y,z,w;
};

struct Vec2i {
    int x,y;
};

struct Mat4r {
    // Stored in row-major order: m[row][column]
    real m[4][4];

    // Default constructor: Create an Identity Matrix
    // An identity matrix does nothing (like multiplying by 1)
    Mat4r() {
        m[0][0] = 1; m[0][1] = 0; m[0][2] = 0; m[0][3] = 0;
        m[1][0] = 0; m[1][1] = 1; m[1][2] = 0; m[1][3] = 0;
        m[2][0] = 0; m[2][1] = 0; m[2][2] = 1; m[2][3] = 0;
        m[3][0] = 0; m[3][1] = 0; m[3][2] = 0; m[3][3] = 1;
    }

    // Constructor from 16 row-major values
    Mat4r(const real values[16]) {
        m[0][0] = values[0];  m[0][1] = values[1];  m[0][2] = values[2];  m[0][3] = values[3];
        m[1][0] = values[4];  m[1][1] = values[5];  m[1][2] = values[6];  m[1][3] = values[7];
        m[2][0] = values[8];  m[2][1] = values[9];  m[2][2] = values[10]; m[2][3] = values[11];
        m[3][0] = values[12]; m[3][1] = values[13]; m[3][2] = values[14]; m[3][3] = values[15];
    }

    // --- You will add functions to this struct ---

    // Function to calculate and return the inverse of this matrix
    Mat4r inverse() const {
        // Convert to glm (our Mat4r is row-major, glm is column-major)
        glm::mat<4,4,real,glm::defaultp> gm;
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
                gm[c][r] = m[r][c];   // transpose copy

        // Compute inverse with GLM
        glm::mat<4,4,real,glm::defaultp> invGm = glm::inverse(gm);

        // Convert back to Mat4r (column-major → row-major)
        Mat4r inv;
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
                inv.m[r][c] = invGm[c][r];

        return inv;
    }

    // Function to calculate and return the transpose of this matrix
    Mat4r transpose() const {
        Mat4r trans;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                trans.m[i][j] = m[j][i];
            }
        }
        return trans;
    }

    glm::mat<4,4,real,glm::defaultp> toGlm() const {
        glm::mat<4,4,real,glm::defaultp> gm;
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
                gm[c][r] = m[r][c];
        return gm;
    }

    static Mat4r fromGlm(const glm::mat<4,4,real,glm::defaultp>& gm) {
        Mat4r out;
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
                out.m[r][c] = gm[c][r];
        return out;
    }

    // --- And you'll add static "factory" functions ---

    static Mat4r identity() {
        return Mat4r(); // Default constructor is identity
    }

    static Mat4r createTranslation(real tx, real ty, real tz) {
        glm::mat4 gm = glm::translate(glm::mat4(1.0), glm::vec3(tx, ty, tz));
        return fromGlm(gm);
    }

    static Mat4r createScaling(real sx, real sy, real sz) {
        glm::mat4 gm = glm::scale(glm::mat4(1.0), glm::vec3(sx, sy, sz));
        return fromGlm(gm);
    }

    Mat4r operator*(const Mat4r& rhs) const {
        return fromGlm(toGlm() * rhs.toGlm());
    }
    
    static Mat4r createRotation(real angleRadians, real axisX, real axisY, real axisZ) {
        glm::mat<4,4,real,glm::defaultp> gm = glm::rotate(
            glm::mat<4,4,real,glm::defaultp>(1.0),
            angleRadians,
            glm::vec<3,real>(axisX, axisY, axisZ)
        );
        return fromGlm(gm);
    }
};

