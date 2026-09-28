#include "BVH.h"
#include "Scene.h"

void BVH::build(Mesh& mesh) {
    currentMesh = &mesh;
    size_t triCount = mesh.getTriangles().size();
    if (triCount == 0) {
        return;
    }
    
    primIndices.resize(triCount);
    for (size_t i = 0; i < triCount; i++)
        primIndices[i] = i;   

    nodes.clear();
    nodes.reserve(triCount * 2); 

    buildRecursive(0, triCount);
}

uint BVH::buildRecursive(uint first, uint count) {
    BVHNode node;
    Axis axis = pickSplitAxis(first, count, node);

    uint nodeIndex = nodes.size();
    nodes.push_back(node);

    if (count <= 10) {
        nodes[nodeIndex].firstPrim = first;
        nodes[nodeIndex].primCount = count;
        return nodeIndex;
    } 
    nodes[nodeIndex].primCount = 0;

    uint mid = first + count / 2;
    splitByMedian(first, count, axis);

    nodes[nodeIndex].left_child  = buildRecursive(first, count / 2);
    nodes[nodeIndex].right_child = buildRecursive(mid, count - count / 2);

    return nodeIndex;
}

// Using nth element method which is O(N)
void BVH::splitByMedian(uint first, uint count, Axis axis) {
    uint mid = first + count / 2;

    auto& tris = currentMesh->getTriangles();

    switch (axis) {
        case X_AXIS:
            std::nth_element(primIndices.begin() + first,
                             primIndices.begin() + mid,
                             primIndices.begin() + first + count,
                [&](uint ia, uint ib) {
                    return tris[ia].getCentroid().x < tris[ib].getCentroid().x;
                });
            break;

        case Y_AXIS:
            std::nth_element(primIndices.begin() + first,
                             primIndices.begin() + mid,
                             primIndices.begin() + first + count,
                [&](uint ia, uint ib) {
                    return tris[ia].getCentroid().y < tris[ib].getCentroid().y;
                });
            break;

        case Z_AXIS:
            std::nth_element(primIndices.begin() + first,
                             primIndices.begin() + mid,
                             primIndices.begin() + first + count,
                [&](uint ia, uint ib) {
                    return tris[ia].getCentroid().z < tris[ib].getCentroid().z;
                });
            break;
    }
}

Axis BVH::pickSplitAxis(uint first, uint count, BVHNode& node) {
    Vec3r minV(FLT_MAX), maxV(-FLT_MAX);
    auto& tris = currentMesh->getTriangles();

    for (uint i = first; i < first + count; i++) {
        const Triangle& t = tris[primIndices[i]];
        const Vec3r& v0 = scene->vertex_data[t.getV0ID()-1];
        const Vec3r& v1 = scene->vertex_data[t.getV1ID()-1];
        const Vec3r& v2 = scene->vertex_data[t.getV2ID()-1];

        minV.x = std::min({minV.x, v0.x, v1.x, v2.x});
        minV.y = std::min({minV.y, v0.y, v1.y, v2.y});
        minV.z = std::min({minV.z, v0.z, v1.z, v2.z});

        maxV.x = std::max({maxV.x, v0.x, v1.x, v2.x});
        maxV.y = std::max({maxV.y, v0.y, v1.y, v2.y});
        maxV.z = std::max({maxV.z, v0.z, v1.z, v2.z});
    }

    node.aabb_min = minV;
    node.aabb_max = maxV;

    Vec3r extent = maxV - minV;

    if (extent.x >= extent.y && extent.x >= extent.z) return X_AXIS;
    if (extent.y >= extent.z) return Y_AXIS;
    return Z_AXIS;
}

bool BVH::intersect(Ray& r, HitRecord& rec) const {
    rec.t = INFINITY;
    return intersectNode(0, r, rec); // root is node 0
}

bool BVH::intersectNode(uint nodeIndex, Ray& r, HitRecord& rec) const {
    BVHNode node = nodes[nodeIndex];

    if (!rayAABB(r, node.aabb_min, node.aabb_max, rec.t)) {
        return false;   
    }   

    // Leaf Node
    if (node.primCount > 0) {
        bool hit = false;
        auto& tris = currentMesh->getTriangles();

        // Store the hit record
        for (uint i = 0; i < node.primCount; i++) {
            uint triIdx = primIndices[node.firstPrim + i];
            Triangle& tri = tris[triIdx];

            real t = tri.checkTriangleIntersection(
                r,
                scene->vertex_data[tri.getV0ID() - 1],
                scene->vertex_data[tri.getV1ID() - 1],
                scene->vertex_data[tri.getV2ID() - 1],
                currentMesh->getShadingMode()
            );

            if (t < rec.t) {
                r.tri_idx = triIdx;
                rec.t = t;
                rec.material_id = tri.getMaterialID();

                Vec3r p = r.start_point + r.direction * t;
                rec.intersection_point = p;

                if (currentMesh->getShadingMode() == "flat")
                    rec.normal = tri.getNormal();
                else
                    rec.normal = tri.getShadingNormal();

                hit = true;
            }
        }
        
        return hit;
    }

    bool hitLeft  = intersectNode(node.left_child, r, rec);
    bool hitRight = intersectNode(node.right_child, r, rec);

    return hitLeft || hitRight;
}

bool BVH::rayAABB(Ray& r, Vec3r& minB, Vec3r& maxB, real tMax) const
{
    Vec3r invDir = {1.0 / r.direction.x, 1.0 / r.direction.y, 1.0 / r.direction.z}; 

    real tmin = (minB.x - r.start_point.x) * invDir.x;
    real tmax = (maxB.x - r.start_point.x) * invDir.x;
    if (tmin > tmax) std::swap(tmin, tmax);

    real tymin = (minB.y - r.start_point.y) * invDir.y;
    real tymax = (maxB.y - r.start_point.y) * invDir.y;
    if (tymin > tymax) std::swap(tymin, tymax);

    if (tmin > tymax || tymin > tmax)
        return false;

    if (tymin > tmin) tmin = tymin;
    if (tymax < tmax) tmax = tymax;

    real tzmin = (minB.z - r.start_point.z) * invDir.z;
    real tzmax = (maxB.z - r.start_point.z) * invDir.z;
    if (tzmin > tzmax) std::swap(tzmin, tzmax);

    if (tmin > tzmax || tzmin > tmax)
        return false;

    if (tzmin > tmin) tmin = tzmin;
    if (tzmax < tmax) tmax = tzmax;

    return tmin < tMax && tmax > 0.0;
}



