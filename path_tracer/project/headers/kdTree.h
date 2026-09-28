#pragma once

#include "Real.h"
#include "Utils.h"

struct Scene;
class Mesh;

enum class SplitMethod {
    SpatialMedian, 
    ObjectMedian,  
    SAH            
};

struct KDPrimitive {
    TYPE type;
    int id;        
    int mesh_id;   
};

struct BuildPrimitive {
    KDPrimitive ref; 
    Vec3r min;      
    Vec3r max;       
    Vec3r center;    
};

struct KDNode {
    int split_axis = -1; 
    real split_pos = 0;
    
    int left_child = -1; 
    int right_child = -1;

    std::vector<KDPrimitive> primitives; 
};

class KDTree {
public:
    void build(Scene& scene, SplitMethod method);
    bool intersectAABB(const Ray& ray, real& t_enter, real& t_exit);

    const std::vector<KDNode>& getNodes() const { return nodes; }
    const Vec3r& getRootMin() const { return root_min; }
    const Vec3r& getRootMax() const { return root_max; }

    void buildFromMesh(Mesh& mesh, const Scene& scene, SplitMethod method);
private:
    std::vector<KDNode> nodes;
    Vec3r root_min;
    Vec3r root_max;
    
    int buildRecursive(std::vector<BuildPrimitive>& objects, 
                       const Vec3r& node_min, const Vec3r& node_max, 
                       int depth, SplitMethod method);

    real findObjectMedian(std::vector<BuildPrimitive>& objects, int axis);
};