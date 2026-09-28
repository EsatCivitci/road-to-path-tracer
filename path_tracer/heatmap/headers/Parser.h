#pragma once

#include "Scene.h"

class Parser {
public:    
    Parser();
    ~Parser();

    void parseScene(const std::string& filename, Scene& scene);
    void checkParsedScene(Scene& scene);
private:
    Vec3r parseVec3r(const std::string& s);
    Vec2i parseVec2i(const std::string& s);
    Vec2r parseVec2r(const std::string& s);
    std::vector<real> parseNearPlane(const std::string& s);
    bool loadPlyMesh(const std::string& filename, std::vector<Vec3r>& vertices, std::vector<Vec3i>& triangles, std::vector<Vec3r>& normals, std::vector<Vec2r>& texCoords);
    void computeVertexNormals(const std::vector<Vec3r>& vertices, std::vector<Triangle>& triangles);
};