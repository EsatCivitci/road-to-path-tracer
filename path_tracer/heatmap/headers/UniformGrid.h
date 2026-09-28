#pragma once

#include "Real.h"
#include "Utils.h"

struct Scene;
struct Ray;

struct gObject {
    int id = -1;
    int mesh_id = -1;
    TYPE obj_type;
    Vec3r aabb_min {  FLT_MAX,  FLT_MAX,  FLT_MAX };
    Vec3r aabb_max { -FLT_MAX, -FLT_MAX, -FLT_MAX };
};

struct uGrid {
    Vec3r aabb_min {  FLT_MAX,  FLT_MAX,  FLT_MAX };
    Vec3r aabb_max { -FLT_MAX, -FLT_MAX, -FLT_MAX };
    std::vector<gObject> objects;
};

class UniformGrid {
public:
    UniformGrid() = default;
    ~UniformGrid() = default;

    bool buildGrid(Scene& scene);
    bool intersectAABB(Ray& ray, real& t_enter, real& t_exit);

    const uGrid getMainGrid() const { return main_grid; }
    const std::vector<std::vector<int>> getGridList() const {return grid_lst;}
    Vec3r getCellSize() { return cell_size; }
    int getNX() {return nx;}
    int getNY() {return ny;}
    int getNZ() {return nz;}

private:
    bool computeMainGrid(Scene& scene);

    uGrid main_grid;
    std::vector<std::vector<int>> grid_lst;

    int nx, ny, nz;     
    Vec3r cell_size;
};