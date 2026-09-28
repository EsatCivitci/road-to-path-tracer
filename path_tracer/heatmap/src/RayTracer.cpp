#include "RayTracer.h"
#include <thread>
#include <random>
#include <algorithm>
#include "cmath" 
#include "tinyexr.h"


#include "stb_image_write.h"
// #define STB_IMAGE_IMPLEMENTATION
// #include "stb_image.h"
#include "BVH.h"
#include "TransformUtils.h"

RayTracer::RayTracer (Scene& input_scene) : scene(input_scene) {}

RayTracer::~RayTracer() {}

void RayTracer::startRayTracer() {
    for (const auto& camera : scene.cameras) {
        Vec2i image_resolution = camera.getImageResolution();
        int nx = image_resolution.x;
        int ny = image_resolution.y;

        renderer = camera.getRenderer();

        std::vector<unsigned char> image_data(nx * ny * 3);
        std::vector<real> color_data(nx * ny * 3);

        int nSamples = camera.getNumSamples();
        int sample_length = sqrt(nSamples);

        for (auto& al : scene.area_lights) {
            al.nSamples = nSamples;
        }


        std::vector<random9r> samples;
        std::vector<real> gaussian_weights;
        if (camera.getNumSamples() != -1) {
            samples.reserve(nSamples);
            gaussian_weights.reserve(nSamples);
            generateRandoms(sample_length);
            samples = scene.samples;
            gaussian_weights = precomputeGaussianWeights(nSamples);
        }
        else {
            samples.clear();
            gaussian_weights.clear();
        }

        std::vector<real> near_plane = camera.getNearPlane();
        real l = near_plane[0];
        real r = near_plane[1];
        real b = near_plane[2];
        real t = near_plane[3];

        Vec3r gaze = camera.getGaze();
        Vec3r up = camera.getUp();
        Vec3r right;
        if (!camera.isTransformed) {           
            gaze = normalizeVec3r(camera.getGaze());
            up = normalizeVec3r(camera.getUp());

            right = normalizeVec3r(crossVec3r(gaze, up)); 
            up = normalizeVec3r(crossVec3r(right, gaze));
        }
        else {            

            right = normalizeVec3r(crossVec3r(gaze, up)); 
        }

        Vec3r position = camera.getPosition();
        real near_distance = camera.getNearDistance();

        Vec3r m = position + gaze * near_distance;
        Vec3r q = m + l * right + t * up;


        ToneMap tone_map = camera.getTM();
        bool isTMActive = tone_map.isActive();   


        Ray ray;
        ray.start_point = position;
        for (int i = 0; i < nx; ++i) { 
            for (int j = 0; j < ny; ++j) { 
                Vec3r color = {0,0,0};
                int numSamples = camera.getNumSamples();
                if (numSamples != -1) {
                    for (int k = 0; k < nSamples; ++k) {
                        real s_u = (i + samples[k].psi_1) * (r - l) / float(nx);
                        real s_v = (j + samples[k].psi_2) * (t - b) / float(ny);

                        Vec3r s = q + s_u * right - s_v * up;

                        if (camera.getApertureSize() != -1) {
                            Vec3r dir = normalizeVec3r(s - position);

                            real t_fd = camera.getFocusDistance() / dotVec3r(dir, gaze);
                            Vec3r p = position + t_fd * dir;

                            ray.start_point = position + camera.getApertureSize() * (right * (samples[k].psi_3 - 0.5) + up * (samples[k].psi_4 - 0.5));
                            ray.direction = normalizeVec3r(p - ray.start_point);
                            ray.depth = 0;
                            ray.renderer = renderer;
                        }
                        else {
                            ray.direction = normalizeVec3r(s - position);
                            ray.depth = 0;
                            ray.renderer = renderer;
                        }

                        curr_sample_index = k;

                        color = color + computeColor(ray); //* gaussian_weights[k];
                    }
                    color = color / nSamples; // Box Filter
                }
                else {
                    real s_u = (i + 0.5) * (r - l) / float(nx);
                    real s_v = (j + 0.5) * (t - b) / float(ny);
                    
                    Vec3r s = q + s_u * right - s_v * up;

                    ray.direction = normalizeVec3r(s - position);
                    ray.depth = 0;
                    ray.renderer = renderer;

                    color = color + computeColor(ray);
                }
                if (!isTMActive) color = clampVec3r(color, 0, 255);
                int index = (j * nx + i) * 3;

                color_data[index + 0] = color.x;
                color_data[index + 1] = color.y;
                color_data[index + 2] = color.z;

            }
        }


        std::cout << "Rendering is done!!" << std::endl;

        if (isTMActive) {
            int pos = camera.getImageName().find(".");
            std::string name = camera.getImageName().substr(0, pos);
            tone_map.applyToneMapping(color_data, name, nx, ny);

            std::vector<float> f_data(nx * ny * 3);
            for (int i = 0; i < nx * ny * 3; ++i) {
                f_data[i] = static_cast<float>(color_data[i]);
            }

            std::string output_path = "my_outputs/" + camera.getImageName();
            const char* err = nullptr;
            int ret = SaveEXR(f_data.data(), nx, ny, 3, 0, output_path.c_str(), &err);
        } 
        else {
            int image_size = nx * ny * 3;
            for (int i = 0; i < image_size; ++i) {
                image_data[i] = static_cast<unsigned char>(color_data[i]);
            }

            // Generate image!!
            int stride_bytes = nx * 3; 
            std::string output_path = "my_outputs/" + camera.getImageName();
            stbi_write_png(output_path.c_str(), nx, ny, 3, image_data.data(), stride_bytes);
        }

        samples.clear();
        gaussian_weights.clear();


    }
}

/*
        unsigned int num_threads = std::thread::hardware_concurrency();
        std::vector<std::thread> threads;
        int rows_per_thread = ny / num_threads;

        // --- 2. Launch threads ---
        for (unsigned int t_idx = 0; t_idx < num_threads; ++t_idx) {
            int start_row = t_idx * rows_per_thread;
            int end_row = (t_idx == num_threads - 1) ? ny : start_row + rows_per_thread;

            threads.emplace_back([&, start_row, end_row] {
                // This is the worker code each thread will run
                Ray ray;
                ray.start_point = position;
                for (int j = start_row; j < end_row; ++j) {
                    for (int i = 0; i < nx; ++i) {
                        Vec3r color = {0,0,0};
                        int numSamples = camera.getNumSamples();
                        if (numSamples != -1) {
                            for (int k = 0; k < nSamples; ++k) {
                                real s_u = (i + samples[k].psi_1) * (r - l) / float(nx);
                                real s_v = (j + samples[k].psi_2) * (t - b) / float(ny);

                                Vec3r s = q + s_u * right - s_v * up;

                                if (camera.getApertureSize() != -1) {
                                    Vec3r dir = normalizeVec3r(s - position);

                                    real t_fd = camera.getFocusDistance() / dotVec3r(dir, gaze);
                                    Vec3r p = position + t_fd * dir;

                                    ray.start_point = position + camera.getApertureSize() * (right * (samples[k].psi_3 - 0.5) + up * (samples[k].psi_4 - 0.5));
                                    ray.direction = normalizeVec3r(p - ray.start_point);
                                    ray.depth = 0;
                                }
                                else {
                                    ray.direction = normalizeVec3r(s - position);
                                    ray.depth = 0;
                                }

                                curr_sample_index = k;

                                color = color + computeColor(ray); //* gaussian_weights[k];
                            }
                            color = color / nSamples; // Box Filter
                        }
                        else {
                            real s_u = (i + 0.5) * (r - l) / float(nx);
                            real s_v = (j + 0.5) * (t - b) / float(ny);
                            
                            Vec3r s = q + s_u * right - s_v * up;

                            ray.direction = normalizeVec3r(s - position);
                            ray.depth = 0;

                            color = color + computeColor(ray);
                        }
                        if (color.x == -1) {
                            const TextureMap texMap = scene.texture.getTexMap(scene.texture.getBackgroundTextureID() - 1);
                            const Image& texImg = scene.texture.getImage(texMap.image_ID - 1);
                            color = applyBackgroundTexture(i, j, nx, ny, texMap, texImg);
                        }
                        if (!isTMActive) color = clampVec3r(color, 0, 255);
                        int index = (j * nx + i) * 3;

                        color_data[index + 0] = color.x;
                        color_data[index + 1] = color.y;
                        color_data[index + 2] = color.z;
                    }
                }
            });
        }

        // --- 3. Wait for all threads to finish ---
        for (auto& t : threads) {
            t.join();
        }

        Ray ray;
        ray.start_point = position;
        for (int i = 0; i < nx; ++i) { 
            for (int j = 0; j < ny; ++j) { 
                Vec3r color = {0,0,0};
                int numSamples = camera.getNumSamples();
                if (numSamples != -1) {
                    for (int k = 0; k < nSamples; ++k) {
                        real s_u = (i + samples[k].psi_1) * (r - l) / float(nx);
                        real s_v = (j + samples[k].psi_2) * (t - b) / float(ny);

                        Vec3r s = q + s_u * right - s_v * up;

                        if (camera.getApertureSize() != -1) {
                            Vec3r dir = normalizeVec3r(s - position);

                            real t_fd = camera.getFocusDistance() / dotVec3r(dir, gaze);
                            Vec3r p = position + t_fd * dir;

                            ray.start_point = position + camera.getApertureSize() * (right * (samples[k].psi_3 - 0.5) + up * (samples[k].psi_4 - 0.5));
                            ray.direction = normalizeVec3r(p - ray.start_point);
                            ray.depth = 0;
                            ray.renderer = renderer;
                        }
                        else {
                            ray.direction = normalizeVec3r(s - position);
                            ray.depth = 0;
                            ray.renderer = renderer;
                        }

                        curr_sample_index = k;

                        color = color + computeColor(ray); //* gaussian_weights[k];
                    }
                    color = color / nSamples; // Box Filter
                }
                else {
                    real s_u = (i + 0.5) * (r - l) / float(nx);
                    real s_v = (j + 0.5) * (t - b) / float(ny);
                    
                    Vec3r s = q + s_u * right - s_v * up;

                    ray.direction = normalizeVec3r(s - position);
                    ray.depth = 0;
                    ray.renderer = renderer;

                    color = color + computeColor(ray);
                }
                if (!isTMActive) color = clampVec3r(color, 0, 255);
                int index = (j * nx + i) * 3;

                color_data[index + 0] = color.x;
                color_data[index + 1] = color.y;
                color_data[index + 2] = color.z;

            }
        }
*/

Vec3r RayTracer::computeColor(Ray& ray) {
    HitRecord hit_record;
    if (ray.depth > scene.max_recursion_depth) {
        // return Vec3r(0.0);
        return computeHeatMapColor(0);
    }
    // if (closestHitKDTree(ray, hit_record)) {
    // if (closestHitGrid(ray, hit_record)) {
    if (closestHit(ray, hit_record)) {
        // Material mat = scene.materials[hit_record.material_id - 1];
        // if (ray.renderer == "PathTracing") return pathTracing(ray, hit_record);
        // else {
        //     Vec3r color =  applyShading(ray, hit_record);
        //     return color;
        // } 
        return computeHeatMapColor(hit_record.traversal_steps);
    }
    else return computeHeatMapColor(0);
    // else if (ray.depth == 0) {
        // if (scene.texture.isBackgroundTexture()) {
        //     return {-1, 0, 0};
        // }
        // else if (scene.env_lights.size() > 0) {
        //     Vec3r d = normalizeVec3r(ray.direction);
        //     for (const auto& env_light : scene.env_lights) {
        //         Image tex_img = scene.texture.getImage(env_light.getImageID() - 1);
        //         return computeEnvironmentColor(tex_img, d, env_light.getType());
        //     }
        // }
        // else {
        //     return scene.background_color;
        // }
    //     return computeHeatMapColor(0);
    // }
    // else {
    //     return {0, 0, 0};
    // }
}

bool RayTracer::closestHitKDTree(Ray& ray, HitRecord& hit_record) {
    hit_record.t = std::numeric_limits<real>::infinity();
    bool hitAnything = false;

    for (auto& plane : scene.planes) {
        if (processPlaneHit(ray, plane, hit_record)) hitAnything = true;
    }

    real t_enter, t_exit;
    if (!scene.kd_tree.intersectAABB(ray, t_enter, t_exit)) {
        return hitAnything;
    }

    const auto& nodes = scene.kd_tree.getNodes();
    // std::cout << nodes.size() << std::endl;
    std::function<void(int, real, real)> traverse = 
        [&](int node_idx, real t0, real t1) {

        if (t0 >= hit_record.t) return;

        hit_record.traversal_steps++;

        // std::cout << node_idx << std::endl;
        const KDNode& node = nodes[node_idx];
        if (node.split_axis == -1) {
            hit_record.traversal_steps += node.primitives.size() * 2;
            for (const auto& prim : node.primitives) {
                switch (prim.type) {
                    case TYPE::Sphere:
                        if (processSphereHit(ray, scene.spheres[prim.id-1], hit_record)) hitAnything = true;
                        break;

                    case TYPE::Triangle:
                        if (processTriangleHit(ray, scene.triangles[prim.id-1], hit_record)) hitAnything = true;
                        break;

                    case TYPE::Mesh:
                    {
                        Mesh& mesh = scene.meshes[prim.mesh_id-1];
                        if (processGridMeshHit(ray, mesh, prim.id, hit_record)) hitAnything = true;
                    }
                    break;
                    
                    case TYPE::MeshInstance:
                    {
                        MeshInstance& inst = scene.mesh_instances[prim.mesh_id-1];
                        int base_idx = getMeshByID(scene, inst.baseMeshObjectId);
                        Mesh& baseMesh = scene.meshes[base_idx];
                        
                        if (processGridMeshInstanceHit(ray, inst, baseMesh, prim.id, hit_record)) hitAnything = true;
                    }
                    break;
                    
                    case TYPE::LightSphere:
                         if (processLightSphereHit(ray, scene.light_spheres[prim.id], hit_record)) hitAnything = true;
                         break;

                    default: break;
                }
            }
            return;
        }

        int axis = node.split_axis;
        real dist = (node.split_pos - ray.start_point[axis]) / ray.direction[axis];

        int near_child = (ray.direction[axis] >= 0) ? node.left_child : node.right_child;
        int far_child  = (ray.direction[axis] >= 0) ? node.right_child : node.left_child;

        if (dist >= t1) { 
            traverse(near_child, t0, t1);
        }
        else if (dist <= t0) {
            traverse(far_child, t0, t1);
        }
        else {
            traverse(near_child, t0, dist);

            if (hit_record.t >= dist) {
                traverse(far_child, dist, t1);
            }
        }
    };

    if (t_enter < 0) t_enter = 0;
    traverse(0, t_enter, t_exit);

    return hitAnything;
}

bool RayTracer::closestHitGrid(Ray& ray, HitRecord& hit_record) {
    UniformGrid& grid = scene.grid;
    hit_record.t = std::numeric_limits<real>::infinity();
    bool hitAnything = false;


    for (auto& plane : scene.planes) {
        if (processPlaneHit(ray, plane, hit_record)) hitAnything = true;
    }

    real t_enter, t_exit;
    if (!grid.intersectAABB(ray, t_enter, t_exit)) {
        hit_record.traversal_steps++;
        return hitAnything; 
    }

    if (t_enter < 0) t_enter = 0;

    const uGrid& main_grid = grid.getMainGrid();
    Vec3r cell_size = grid.getCellSize();
    
    Vec3r p = ray.start_point + t_enter * ray.direction;
    int x = clamp(static_cast<int>((p.x - main_grid.aabb_min.x) / cell_size.x), 0, grid.getNX() - 1);
    int y = clamp(static_cast<int>((p.y - main_grid.aabb_min.y) / cell_size.y), 0, grid.getNY() - 1);
    int z = clamp(static_cast<int>((p.z - main_grid.aabb_min.z) / cell_size.z), 0, grid.getNZ() - 1);

    int stepX = (ray.direction.x > 0) ? 1 : -1;
    int stepY = (ray.direction.y > 0) ? 1 : -1;
    int stepZ = (ray.direction.z > 0) ? 1 : -1;

    Vec3r inv_dir = 1.0 / ray.direction;

    real tDeltaX = (stepX > 0) ? cell_size.x * inv_dir.x : cell_size.x * (-inv_dir.x);
    real tDeltaY = (stepY > 0) ? cell_size.y * inv_dir.y : cell_size.y * (-inv_dir.y);
    real tDeltaZ = (stepZ > 0) ? cell_size.z * inv_dir.z : cell_size.z * (-inv_dir.z);

    if (std::abs(ray.direction.x) < 1e-6) tDeltaX = std::numeric_limits<real>::infinity();
    if (std::abs(ray.direction.y) < 1e-6) tDeltaY = std::numeric_limits<real>::infinity();
    if (std::abs(ray.direction.z) < 1e-6) tDeltaZ = std::numeric_limits<real>::infinity();

    real tMaxX = (stepX > 0) 
        ? t_enter + (main_grid.aabb_min.x + (x + 1) * cell_size.x - p.x) * inv_dir.x
        : t_enter + (p.x - (main_grid.aabb_min.x + x * cell_size.x)) * (-inv_dir.x);

    real tMaxY = (stepY > 0) 
        ? t_enter + (main_grid.aabb_min.y + (y + 1) * cell_size.y - p.y) * inv_dir.y
        : t_enter + (p.y - (main_grid.aabb_min.y + y * cell_size.y)) * (-inv_dir.y);

    real tMaxZ = (stepZ > 0) 
        ? t_enter + (main_grid.aabb_min.z + (z + 1) * cell_size.z - p.z) * inv_dir.z
        : t_enter + (p.z - (main_grid.aabb_min.z + z * cell_size.z)) * (-inv_dir.z);

    
    const std::vector<std::vector<int>>& grid_lst = grid.getGridList();
    while (true) {
        int cell_index = x + (y * grid.getNX()) + (z * grid.getNX() * grid.getNY());
        
        hit_record.traversal_steps++;

        if (cell_index >= 0 && cell_index < grid_lst.size()) {
            
            const auto& object_indices = grid_lst[cell_index];
            
            for (int obj_idx : object_indices) {
                const gObject& obj = main_grid.objects[obj_idx];

                hit_record.traversal_steps += object_indices.size() * 2;

                hit_record.traversal_steps++;

                switch (obj.obj_type) {
                    case TYPE::Sphere:
                        if (processSphereHit(ray, scene.spheres[obj.id - 1], hit_record)) hitAnything = true;
                        break;
                    
                    case TYPE::Triangle:
                        if (processTriangleHit(ray, scene.triangles[obj.id - 1], hit_record)) hitAnything = true;
                        break;
                    
                    case TYPE::Mesh:
                        if (processGridMeshHit(ray, scene.meshes[obj.mesh_id - 1], obj.id, hit_record)) hitAnything = true;
                        break;

                    case TYPE::LightSphere:
                        if (processLightSphereHit(ray, scene.light_spheres[obj.id - 1], hit_record)) hitAnything = true;
                        break;
                    
                    case TYPE::MeshInstance:
                        {
                            MeshInstance& inst = scene.mesh_instances[obj.mesh_id - 1];
                            int base_mesh_idx = getMeshByID(scene, inst.baseMeshObjectId); 
                            Mesh& baseMesh = scene.meshes[base_mesh_idx];
                            int triangle_idx = obj.id; 
                            if (processGridMeshInstanceHit(ray, inst, baseMesh, triangle_idx, hit_record)) {
                                hitAnything = true;
                            }
                        }
                        break;
                    

                    default: break;
                }
            }
        }
        real t_next_boundary = std::min({tMaxX, tMaxY, tMaxZ});
        
        if (hitAnything && hit_record.t < t_next_boundary) {
            return true;
        }

        if (tMaxX < tMaxY) {
            if (tMaxX < tMaxZ) {
                x += stepX;
                if (x < 0 || x >= grid.getNX()) break;
                tMaxX += tDeltaX;
            } else {
                z += stepZ;
                if (z < 0 || z >= grid.getNZ()) break;
                tMaxZ += tDeltaZ;
            }
        } else {
            if (tMaxY < tMaxZ) {
                y += stepY;
                if (y < 0 || y >= grid.getNY()) break;
                tMaxY += tDeltaY;
            } else {
                z += stepZ;
                if (z < 0 || z >= grid.getNZ()) break;
                tMaxZ += tDeltaZ;
            }
        }
    }

    return hitAnything;
}

bool RayTracer::closestHit(Ray& ray, HitRecord& hit_record) {
    hit_record.t = std::numeric_limits<real>::infinity(); 
    bool hitAnything = false;
    real distance = 0;
    
    for (auto& plane : scene.planes) {
        if (processPlaneHit(ray, plane, hit_record)) hitAnything = true;
    }

    for (auto& sphere : scene.spheres) {
         if (processSphereHit(ray, sphere, hit_record)) hitAnything = true;
    }
    
    for (auto& triangle : scene.triangles) {
        if (processTriangleHit(ray, triangle, hit_record)) hitAnything = true;
    }

    for (auto& mesh : scene.meshes) {
        if (processMeshHit(ray, mesh, hit_record)) hitAnything = true;
    }

    for (auto& inst : scene.mesh_instances) {
        if (processMeshInstanceHit(ray, inst, hit_record)) hitAnything = true;
    }

    for (auto& lSphere : scene.light_spheres) {
        if (processLightSphereHit(ray, lSphere, hit_record)) hitAnything = true;
    }

    for (auto& lMesh : scene.light_meshes) {
        if (processLightMeshHit(ray, lMesh, hit_record)) hitAnything = true;
    }

    ray.distance += distance;

    return hitAnything;
}

Vec3r RayTracer::applyShading(Ray& ray, const HitRecord& hit_record) {
    if (hit_record.lightObject == true) return hit_record.object_radiance * 255;
    Material material = scene.materials[hit_record.material_id - 1];
    Vec3r final_color = {0,0,0};
    bool is_entering = dotVec3r(ray.direction, hit_record.normal) < 0;

    Vec3r u;
    Vec3r v;
    if (is_entering) {
        if (hit_record.replaceAll) {
            return hit_record.kd * 255;
        }
        final_color = final_color + material.getAmbientReflectance() * scene.ambient_light;
        for (const auto& point_light : scene.point_lights) {
            Vec3r light_direction = point_light.getLightDirection(hit_record.intersection_point);
            real light_distance = point_light.getLightDistance(light_direction);
            if (!inShadow(hit_record, point_light, light_direction, light_distance)) {
                if (scene.BRDFs.size() > 0) {
                    Vec3r L_i = point_light.getIntensity() / pow(light_distance, 2);
                    Vec3r wi = normalizeVec3r(light_direction);
                    Vec3r wo = -ray.direction;
                    BRDF brdf = scene.BRDFs[material.getBRDFID() - 1];
                    brdf.refractive_index = ray.n;
                    Vec3r fr = computeBRDF(brdf, hit_record.normal, wi, wo, material, hit_record); 
                    real cos_theta = dotVec3r(wi, hit_record.normal);
                    final_color = final_color + L_i * fr * cos_theta;
                }
                else {
                    final_color = final_color + diffuseTerm(hit_record, point_light) + specularTerm(hit_record, point_light, ray);
                }
            }
        }
        
        for (const auto& area_light : scene.area_lights) {
            u = area_light.getU();
            v = area_light.getV();
            real psi_1 = scene.samples[curr_sample_index].psi_6;
            real psi_2 = scene.samples[curr_sample_index].psi_7;
            
            Vec3r point = area_light.getPosition() + area_light.getSize() * (u * (psi_1 - 0.5) + v * (psi_2 - 0.5));
            Vec3r light_direction = area_light.getLightDirection(hit_record.intersection_point, point);
            real light_distance = area_light.getLightDistance(light_direction);
            PointLight pl;
            if (!inShadow(hit_record, pl, light_direction,light_distance)) { 
                if (scene.BRDFs.size() > 0) {
                    Vec3r wi = normalizeVec3r(light_direction);
                    real cos_alpha = dotVec3r(area_light.getNormal(), wi); 
                    if (cos_alpha < 0) cos_alpha = -cos_alpha;  
                    real cos_theta = dotVec3r(wi, hit_record.normal);
                    real area = area_light.getSize() * area_light.getSize();           // Light area
                    Vec3r L_i = (area_light.getRadiance() * area * cos_alpha) / pow(light_distance,2);
                    Vec3r wo = -ray.direction;
                    BRDF brdf = scene.BRDFs[material.getBRDFID() - 1];
                    brdf.refractive_index = ray.n;
                    Vec3r fr = computeBRDF(brdf, hit_record.normal, wi, wo, material, hit_record); 
                    final_color = final_color + L_i * fr * cos_theta;
                }
                else {
                    Vec3r diffuse_color =  diffuseTermArea(hit_record, area_light, point); 
                    Vec3r specular_color = specularTermArea(hit_record, area_light, ray, point);

                    final_color = final_color + diffuse_color + specular_color;
                }

            }
        }
        
        for (const auto& dir_light : scene.dir_lights) {    
            Vec3r light_direction = -dir_light.getDirection();
            real light_distance = std::numeric_limits<real>::infinity();
            PointLight pl;
            if (!inShadow(hit_record, pl, light_direction, light_distance)) {
                final_color = final_color + diffuseTermDirection(hit_record, dir_light) + specularTermDirection(hit_record, dir_light, ray);
            }
        }

        for (const auto& spot_light : scene.spot_lights) {
            Vec3r light_direction;
            real angle;
            if (!spot_light.getLightDirection(hit_record.intersection_point, light_direction, angle)) continue;
            real light_distance = spot_light.getLightDistance(light_direction);
            PointLight pl;
            if (!inShadow(hit_record, pl, light_direction, light_distance)) {
                final_color = final_color + diffuseTermSpot(hit_record, spot_light, light_direction, angle) 
                                          + specularTermSpot(hit_record, spot_light, ray, light_direction, angle);
            }            
        }
        
        for (const auto& env_light : scene.env_lights) {
            if (material.getType() != "default") continue;
            real psi_1 = getRandomFloat();
            real psi_2 = getRandomFloat();

            real pi = 3.14159;

            Vec3r norm = hit_record.normal;
            Vec3r u = computeOrthoBasis(norm);
            Vec3r v = normalizeVec3r(crossVec3r(u, norm));

            Vec3r l;
            if (env_light.getSampler() == "uniform") {
                l = (pow(1 - psi_1 * psi_1, 0.5) * cos(psi_2 * 2 * pi)) * u +
                    (pow(1 - psi_1 * psi_1, 0.5) * sin(psi_2 * 2 * pi)) * v +
                    psi_1 * norm;

                Image tex_img = scene.texture.getImage(env_light.getImageID() - 1);
                Vec3r radiance = computeEnvironmentColor(tex_img, l, env_light.getType()) / (2 * pi);

                final_color = final_color + radiance;
            }
            else { // Cosine
                l = (pow(psi_1, 0.5) * cos(psi_2 * 2 * pi)) * u +
                    (pow(psi_1, 0.5) * sin(psi_2 * 2 * pi)) * v +
                    (pow(1 - psi_1, 0.5)) * norm;

                Image tex_img = scene.texture.getImage(env_light.getImageID() - 1);
                Vec3r radiance = computeEnvironmentColor(tex_img, l, env_light.getType()) * pi;

                final_color = final_color + radiance;
            }
        }
    
        if (scene.BRDFs.size() > 0) {
            for (const auto& lSphere : scene.light_spheres) {
                real psi_1 = getRandomFloat();
                real psi_2 = getRandomFloat();

                Vec3r center = scene.vertex_data[lSphere.getCenterVertexID() - 1];
                Vec3r hit_local = applyTransformToPoint(lSphere.getInverseTransformation(), hit_record.intersection_point);
                Vec3r n = normalizeVec3r(hit_local - center);
                Vec3r u = computeOrthoBasis(n);
                v = normalizeVec3r(crossVec3r(u, n));

                real sin_theta = std::sqrt(1.0f - psi_1 * psi_1);
                real cos_theta = psi_1; 
                real phi = 2.0f * M_PI * psi_2;

                Vec3r l = u * (sin_theta * std::cos(phi)) + 
                          v * (sin_theta * std::sin(phi)) + 
                          n * cos_theta;
                Vec3r light_point = center + (l * lSphere.getRadius());
                light_point = applyTransformToPoint(lSphere.getTransformation(), light_point);

                Vec3r light_vec = light_point - hit_record.intersection_point;
                real dist_sq = dotVec3r(light_vec, light_vec); 
                real dist = std::sqrt(dist_sq); 
                Vec3r wi = light_vec / dist;

                PointLight pl;
                if (!inShadow(hit_record, pl, wi, dist)) {
                    real cos_theta = std::max(0.0, dotVec3r(hit_record.normal, wi));

                    Vec3r wo = -ray.direction;
                    BRDF brdf = scene.BRDFs[material.getBRDFID() - 1];
                    Vec3r fr = computeBRDF(brdf, hit_record.normal, wi, wo, material, hit_record);

                    Vec3r Li = lSphere.getRadiance() / dist_sq;

                    final_color = final_color + Li * fr * cos_theta;
                }
            }
        
            for (auto& lMesh : scene.light_meshes) {
                int num_faces = lMesh.getTriangles().size();
                int random_idx = (int)(getRandomFloat() * num_faces);
                if (random_idx >= num_faces) random_idx = num_faces - 1;

                const auto& tri = lMesh.getTriangles()[random_idx];
                real area = tri.getArea();

                Vec3r v0 = scene.vertex_data[tri.getV0ID() - 1];
                Vec3r v1 = scene.vertex_data[tri.getV1ID() - 1];
                Vec3r v2 = scene.vertex_data[tri.getV2ID() - 1];

                Vec3r edge1 = v1 - v0;
                Vec3r edge2 = v2 - v0;
                Vec3r local_normal = normalizeVec3r(crossVec3r(edge1, edge2));

                Vec3r light_normal_world = normalizeVec3r(applyTransformToVector(lMesh.getInverseTransformation(), local_normal));
                
                // Calculate Area
                real tri_area = 0.5f * lengthSquared(crossVec3r(edge1, edge2));

                real psi_1 = getRandomFloat();
                real psi_2 = getRandomFloat();
                real sqrt_psi1 = std::sqrt(psi_1);
                real u = 1.0f - sqrt_psi1;
                real v = sqrt_psi1 * (1.0f - psi_2);
                real w = sqrt_psi1 * psi_2;

                Vec3r light_point_local = v0 * u + v1 * v + v2 * w;
                Vec3r light_point_world = applyTransformToPoint(lMesh.getTransformation(), light_point_local);

                Vec3r light_vec = light_point_world - hit_record.intersection_point;

                if (dotVec3r(light_vec, light_normal_world) >= 0.0f) {
                    continue; 
                }

                real dist_sq = dotVec3r(light_vec, light_vec);
                real dist = std::sqrt(dist_sq);
                Vec3r wi = light_vec / dist;

                PointLight pl;
                if (!inShadow(hit_record, pl, wi, dist)) {
                    real cos_theta = std::max(0.0, dotVec3r(hit_record.normal, wi));
                    
                    real cos_light = dotVec3r(-wi, light_normal_world); 

                    Vec3r wo = -ray.direction;
                    BRDF brdf = scene.BRDFs[material.getBRDFID() - 1];
                    Vec3r fr = computeBRDF(brdf, hit_record.normal, wi, wo, material, hit_record);
                    Vec3r Li = lMesh.getRadiance() / dist_sq;

                    real weight = area * num_faces;

                    final_color = final_color + Li * fr * cos_theta;
                }
            }
        }
    }
    
    // Othonormal basis for the object
    real roughness = material.getRoughness();
    if (roughness != -1) {
        Vec3r norm = hit_record.normal;
        if (std::abs(norm.x) <= std::abs(norm.y) && std::abs(norm.x) <= std::abs(norm.z)) {
            u.x = 0;
            u.y = -norm.z;
            u.z = norm.y;
        }
        else if (std::abs(norm.y) <= std::abs(norm.x) && std::abs(norm.y) <= std::abs(norm.z)) {
            u.x = -norm.z;
            u.y = 0;
            u.z = norm.x;
        }
        else {
            u.x = -norm.y;
            u.y = norm.x;
            u.z = 0;
        }

        u = normalizeVec3r(u);
        v = normalizeVec3r(crossVec3r(u, norm));
    }

    Vec3r recursive_color = {0, 0, 0};
    Vec3r attenuation = {1,1,1};
    if (!is_entering) {
        attenuation.x = std::exp(-material.getAbsorptionCoefficient().x * ray.distance);
        attenuation.y = std::exp(-material.getAbsorptionCoefficient().y * ray.distance); 
        attenuation.z = std::exp(-material.getAbsorptionCoefficient().z * ray.distance);
    }

    if (material.isMirror()) {
        Ray reflection_ray = reflect(ray, hit_record);
        Vec3r reflection_color = {0,0,0};
        if (roughness != -1) {
            real psi_1 = scene.samples[curr_sample_index].psi_8;
            real psi_2 = scene.samples[curr_sample_index].psi_9;
            reflection_ray.direction = reflection_ray.direction + roughness * (u * (psi_1 - 0.5) + v * (psi_2 - 0.5));
            if (scene.env_lights.size() == 0)
                reflection_color = reflection_color + computeColor(reflection_ray) * material.getMirrorReflectance() * attenuation;
            else {
                EnvironmentLight el = scene.env_lights[0];
                Image tex_img = scene.texture.getImage(el.getImageID() - 1);
                reflection_color = reflection_color + computeEnvironmentColor(tex_img, reflection_ray.direction, el.getType()) * material.getMirrorReflectance() * attenuation;
            } 
        }
        else {
            if (scene.env_lights.size() == 0)
                reflection_color = computeColor(reflection_ray) * material.getMirrorReflectance() * attenuation;
            else {
                EnvironmentLight el = scene.env_lights[0];
                Image tex_img = scene.texture.getImage(el.getImageID() - 1);
                reflection_color = reflection_color + computeEnvironmentColor(tex_img, reflection_ray.direction, el.getType()) * material.getMirrorReflectance() * attenuation;
            } 
        }
        recursive_color = recursive_color + reflection_color;
    }
    else if (material.isConductor()){
        real fresnel = computeFresnelConductor(ray, hit_record);
        Ray reflection_ray = reflect(ray, hit_record);
        Vec3r reflection_color = {0,0,0};
        if (roughness != -1) {
            real psi_1 = scene.samples[curr_sample_index].psi_8;
            real psi_2 = scene.samples[curr_sample_index].psi_9;
            reflection_ray.direction = reflection_ray.direction + roughness * (u * (psi_1 - 0.5) + v * (psi_2 - 0.5));
            reflection_color = reflection_color + computeColor(reflection_ray) * fresnel * material.getMirrorReflectance() * attenuation;
        }
        else {
            reflection_color = computeColor(reflection_ray) * fresnel * material.getMirrorReflectance() * attenuation;
        }
        recursive_color = recursive_color + reflection_color;
    }
    else if (material.isDielectric()) {
        real n1 = is_entering ? 1.0f : ray.n;
        real n2 = is_entering ? material.getRefractionIndex() : 1.0f;

        real reflection_ratio = computeFresnelDielectric(ray, hit_record, n1, n2);
        real transmission_ratio = 1 - reflection_ratio;

        Ray reflection_ray = reflect(ray, hit_record);
        Vec3r reflection_color = {0,0,0};
        if (roughness != -1) {
            real psi_1 = scene.samples[curr_sample_index].psi_8;
            real psi_2 = scene.samples[curr_sample_index].psi_9;
            reflection_ray.direction = reflection_ray.direction + roughness * (u * (psi_1 - 0.5) + v * (psi_2 - 0.5));
            if (scene.env_lights.size() == 0)
                reflection_color = reflection_color + computeColor(reflection_ray) * reflection_ratio;
            else {
                EnvironmentLight el = scene.env_lights[0];
                Image tex_img = scene.texture.getImage(el.getImageID() - 1);
                reflection_color = reflection_color + computeEnvironmentColor(tex_img, reflection_ray.direction, el.getType()) * material.getMirrorReflectance() * attenuation;
            } 
        }
        else {
            if (scene.env_lights.size() == 0)
                reflection_color =  computeColor(reflection_ray) * reflection_ratio;
            else {
                EnvironmentLight el = scene.env_lights[0];
                Image tex_img = scene.texture.getImage(el.getImageID() - 1);
                reflection_color = reflection_color + computeEnvironmentColor(tex_img, reflection_ray.direction, el.getType()) * material.getMirrorReflectance() * attenuation;
            }
        }
        
        Vec3r transmission_color = {0,0,0};
        if (transmission_ratio > 0) {
            Ray transmission_ray = refract(ray, hit_record, n1, n2);
            if (roughness != -1) {
                real psi_1 = scene.samples[curr_sample_index].psi_8;
                real psi_2 = scene.samples[curr_sample_index].psi_9;
                transmission_ray.direction = transmission_ray.direction + roughness * (u * (psi_1 - 0.5) + v * (psi_2 - 0.5));
                transmission_color = transmission_color + computeColor(transmission_ray) * transmission_ratio;
            }
            else {
                transmission_color = computeColor(transmission_ray) * transmission_ratio;
            }
        }
        recursive_color = recursive_color + (reflection_color + transmission_color) * attenuation;
    }

    return recursive_color + final_color;
}

Vec3r RayTracer::pathTracing(Ray& ray, const HitRecord& hit_record) {
    Material material = scene.materials[hit_record.material_id - 1];
    bool is_entering = dotVec3r(ray.direction, hit_record.normal) < 0;

    Vec3r attenuation = {1,1,1};
    if (!is_entering) {
        attenuation.x = std::exp(-material.getAbsorptionCoefficient().x * ray.distance);
        attenuation.y = std::exp(-material.getAbsorptionCoefficient().y * ray.distance); 
        attenuation.z = std::exp(-material.getAbsorptionCoefficient().z * ray.distance);
    }

    Vec3r L_e = hit_record.object_radiance;

    // if (lengthSquared(L_e) > 0) printVec3r(L_e, "rad");

    if (material.isMirror() || material.isConductor()) {
        Ray reflection_ray = reflect(ray, hit_record);
        // reflection_ray.renderer = "";
        Vec3r L_i = computeColor(reflection_ray);

        Vec3r reflectance = material.getMirrorReflectance();
        if (material.isConductor()) {
            reflectance = reflectance * computeFresnelConductor(ray, hit_record);
        }
        return (L_e + L_i * reflectance) * attenuation;
    }
    else if (material.isDielectric()) {
        real n1 = is_entering ? 1.0f : ray.n;
        real n2 = is_entering ? material.getRefractionIndex() : 1.0f;

        real R = computeFresnelDielectric(ray, hit_record, n1, n2);

        if (getRandomFloat() < R) {
            Ray reflection_ray = reflect(ray, hit_record);
            // reflection_ray.renderer = "";
            return (L_e + computeColor(reflection_ray) * material.getMirrorReflectance()) * attenuation;
        } 
        else {
            Ray refraction_ray = refract(ray, hit_record, n1, n2);
            // refraction_ray.renderer = "";
            return (L_e + computeColor(refraction_ray)) * attenuation;
        }
    }
    else {
        real psi_1 = getRandomFloat();
        real psi_2 = getRandomFloat();

        real pi = 3.14159;

        Vec3r norm = hit_record.normal;
        Vec3r u = computeOrthoBasis(norm);
        Vec3r v = normalizeVec3r(crossVec3r(u, norm));

        Vec3r wi;
        wi = (pow(1 - psi_1 * psi_1, 0.5) * cos(psi_2 * 2 * pi)) * u +
             (pow(1 - psi_1 * psi_1, 0.5) * sin(psi_2 * 2 * pi)) * v +
             psi_1 * norm;
        wi = normalizeVec3r(wi);
        float pdf = 1.0f / (2.0f * M_PI);

        if (dotVec3r(wi, norm) < 0) wi = -wi;

        Ray random_ray = reflect(ray, hit_record);
        random_ray.direction = wi;
        // random_ray.renderer = "";
        Vec3r L_i = computeColor(random_ray);

        Vec3r wo = -ray.direction;
        BRDF brdf = scene.BRDFs[material.getBRDFID() - 1];
        brdf.refractive_index = ray.n;
        Vec3r fr = computeBRDF(brdf, norm, wi, wo, material, hit_record);

        float cos_theta = std::max(0.0, dotVec3r(hit_record.normal, wi));
        Vec3r color = L_e + (L_i * fr * cos_theta) / pdf;
        int a = 10;
        return color;
    }
}

Vec3r RayTracer::computeBRDF(BRDF brdf, Vec3r norm, Vec3r wi, Vec3r wo, Material mat, HitRecord hit_record) {
    real pi = 3.14159;
    real cos_theta = dotVec3r(wi, norm);
    if (cos_theta < 0) return Vec3r(0);

    Vec3r wh = (wo + wi) / lengthSquared(wo + wi); 
    Vec3r wi_r = normalizeVec3r(-wi + 2 * norm * cos_theta);
    // printVec3r(wi_r, "wi_r");
    
    real cos_alpha_h = dotVec3r(wh, norm);
    real cos_alpha_r = dotVec3r(wi_r, wo);
    real p = brdf.exponent;

    Vec3r kd = hit_record.kd;
    Vec3r ks = hit_record.ks;

    std::string type = brdf.type;
    if (type == "OriginalBlinnPhong" || type == "default" || type == "") {
        return kd + ks * (pow(cos_alpha_h, p) / cos_theta);
    }
    else if (type == "OriginalPhong") {
        return kd + ks * pow(cos_alpha_r, p) / cos_theta;
    }
    else if (type == "ModifiedBlinnPhong") {
        if (brdf.normalized)
            return kd * (1 / pi) + ks * (p + 8) / (8 * pi) * pow(cos_alpha_h, p);
        else 
            return kd + ks * pow(cos_alpha_h, p);
    }
    else if (type == "ModifiedPhong") {
        if (brdf.normalized) {
            // std::cout << cos_alpha_r << std::endl;
            // printVec3r(ks * (p + 2) / (2 * pi) * pow(cos_alpha_r, p), "");
            return kd * (1 / pi) + ks * (p + 2) / (2 * pi) * pow(cos_alpha_r, p);
        }
        else 
            return kd + ks * pow(cos_alpha_r, p);
    }
    else if (type == "TorranceSparrow") {
        real D = (p + 2) / (2 * pi) * pow(cos_alpha_h, p);
        
        real n_wh = dotVec3r(norm, wh);
        real n_wo = dotVec3r(norm, wo);
        real wo_wh = dotVec3r(wo, wh);
        real n_wi = dotVec3r(norm, wi);

        real G1 = 2 * n_wh * n_wo / wo_wh;
        real G2 = 2 * n_wh * n_wi / wo_wh;
        real G = std::min(1.0, std::min(G1, G2));

        real n = mat.getRefractionIndex();
        real R_0 = pow(n - 1, 2) / pow(n + 1, 2);
        real F = R_0 + (1 - R_0) * pow(1 - wo_wh, 5);

        Vec3r result;
        if (brdf.kd_fresnel) {
            return (1-F) * kd / pi + ks * (D * F * G) / (4 * n_wi * n_wo);
        }
        else {
            return kd / pi + ks * (D * F * G) / (4 * n_wi * n_wo);
        }

    }
}

real RayTracer::computeFresnelDielectric(Ray& ray, const HitRecord& hit_record, real n1, real n2) {
    real cos_theta = dotVec3r(-ray.direction, hit_record.normal);
    if (cos_theta < 0) {
        cos_theta = -cos_theta; 
    }

    real eta = n1 / n2;
    real discriminant = 1 - pow(eta, 2) * (1 - pow(cos_theta, 2));
    real cos_phi;
    if (discriminant > 0) {
        real cos_phi = sqrt(discriminant);

        real r_parallel = (n2 * cos_theta - n1 * cos_phi) / (n2 * cos_theta + n1 * cos_phi);
        real r_perpendicular = (n1 * cos_theta - n2 * cos_phi) / (n1 * cos_theta + n2 * cos_phi);
        return 0.5f * (pow(r_parallel, 2) + pow(r_perpendicular, 2));
    }
    return (real)1;
}

real RayTracer::computeFresnelConductor(Ray& ray, const HitRecord& hit_record) {
    Material mat = scene.materials[hit_record.material_id - 1];
    real cos_theta = std::max((real)0, dotVec3r(-ray.direction, hit_record.normal));
    real n1 = ray.n;
    real n2 = mat.getRefractionIndex();
    real k2 = mat.getAbsorptionIndex();

    real a = pow(n2, 2) + pow(k2, 2);
    real b = 2 * n2 * cos_theta;
    real c = pow(cos_theta, 2);

    real Rs = (a - b + c) / (a + b + c);
    real Rp = (a * c - b + 1) / (a * c + b + 1);

    return 0.5 * (Rs + Rp);
}

Ray RayTracer::reflect(Ray& ray, const HitRecord& hit_record) {
    Vec3r wo = -ray.direction;
    Vec3r normal = hit_record.normal;

    real cos_theta = dotVec3r(wo, normal);
    if (cos_theta < 0) {
        normal = -normal;
        cos_theta = std::max((real)0, dotVec3r(wo, normal));
    }
    
    Ray reflection_ray = ray;
    reflection_ray.direction = normalizeVec3r(-wo + 2 * normal * cos_theta);
    reflection_ray.depth += 1;
    reflection_ray.start_point = hit_record.intersection_point + normal * scene.shadow_ray_epsilon;
    
    return reflection_ray;
}

Ray RayTracer::refract(Ray& ray, const HitRecord& hit_record, real n1, real n2) {
    Vec3r wo = -ray.direction;
    Vec3r normal = hit_record.normal;

    real cos_theta = dotVec3r(wo, normal);
    if (cos_theta < 0) {
        normal = -normal;
        cos_theta = std::max((real)0, dotVec3r(wo, normal));
    }

    real eta = n1 / n2;
    real discriminant = 1 - pow(eta, 2) * (1 - pow(cos_theta, 2));
    if (discriminant > 0) {
        real cos_phi = sqrt(discriminant);

        Ray transmission_ray;
        transmission_ray.direction = normalizeVec3r((-wo + normal * cos_theta) * eta - normal * cos_phi);
        transmission_ray.depth = ray.depth + 1;
        transmission_ray.start_point = hit_record.intersection_point - normal * scene.shadow_ray_epsilon;
        transmission_ray.isIn = !ray.isIn;
        transmission_ray.n = n2;
        if (transmission_ray.isIn) transmission_ray.distance = 0;

        return transmission_ray;
    }
    return {0,0,0};
}

Vec3r RayTracer::diffuseTerm(const HitRecord& hit_record, const PointLight& light) {
    Material material = scene.materials[hit_record.material_id - 1];

    Vec3r light_direction = light.getLightDirection(hit_record.intersection_point);
    real light_distance = light.getLightDistance(light_direction);
    Vec3r wi = normalizeVec3r(light_direction);

    Vec3r irradiance = light.getIntensity() / pow(light_distance,2);
    real cos_theta = std::max((real)0, dotVec3r(wi, hit_record.normal));

    return hit_record.kd * cos_theta * irradiance;
}

Vec3r RayTracer::diffuseTermArea(const HitRecord& hit_record, const AreaLight& light, const Vec3r& point) {
    Material material = scene.materials[hit_record.material_id - 1]; 

    Vec3r light_direction = light.getLightDirection(hit_record.intersection_point, point);
    real light_distance = light.getLightDistance(light_direction);
    Vec3r wi = normalizeVec3r(light_direction);

    real cos_alpha = dotVec3r(light.getNormal(), wi);        // Angle between normal and wi
    if (cos_alpha < 0) cos_alpha = -cos_alpha;               // This line is problematic ask it to Oğuz Hoca
    Vec3r radiance = light.getRadiance();                    // Light radiance
    real area = light.getSize() * light.getSize();           // Light area
    Vec3r irradiance = (radiance * area * cos_alpha) / pow(light_distance,2);

    real cos_theta = std::max((real)0, dotVec3r(wi, hit_record.normal));
    Vec3r color = material.getDiffuseReflectance() * cos_theta * irradiance;
    return material.getDiffuseReflectance() * cos_theta * irradiance;
}

Vec3r RayTracer::diffuseTermDirection(const HitRecord& hit_record, const DirectionalLight& light) {
    Material material = scene.materials[hit_record.material_id - 1];

    Vec3r light_direction = -light.getDirection();
    Vec3r wi = normalizeVec3r(light_direction);

    real cos_theta = std::max((real)0, dotVec3r(wi, hit_record.normal));
    return hit_record.kd * cos_theta * light.getRadiance();
}

Vec3r RayTracer::diffuseTermSpot(const HitRecord& hit_record, const SpotLight& light, Vec3r light_direction, real angle) {
    Material material = scene.materials[hit_record.material_id - 1];

    real light_distance = light.getLightDistance(light_direction);
    Vec3r wi = normalizeVec3r(light_direction);

    real f_angle = light.getFallOffAngle() / 2;
    real c_angle = light.getCoverageAngle() / 2;

    real fall_off_factor = 1;
    if (angle > f_angle) {
        fall_off_factor = pow((cos(toRadians(angle)) - cos(toRadians(c_angle))) / (cos(toRadians(f_angle)) - cos(toRadians(c_angle))), 4);
    }

    Vec3r irradiance = light.getIntensity() / pow(light_distance,2) * fall_off_factor;
    real cos_theta = std::max((real)0, dotVec3r(wi, hit_record.normal));

    return hit_record.kd * cos_theta * irradiance;
}

Vec3r RayTracer::specularTerm(const HitRecord& hit_record, const PointLight& light, Ray& ray) {
    Material material = scene.materials[hit_record.material_id - 1];

    Vec3r light_direction = light.getLightDirection(hit_record.intersection_point);
    real light_distance = light.getLightDistance(light_direction);
    Vec3r wi = normalizeVec3r(light_direction);

    Vec3r irradiance = light.getIntensity() / pow(light_distance,2);
    Vec3r wo = normalizeVec3r(-ray.direction);

    Vec3r h = normalizeVec3r(wi + wo);
    real cos_alpha = std::max((real)0, dotVec3r(hit_record.normal, h));

    return hit_record.ks * pow(cos_alpha, material.getPhongExponent()) * irradiance;
}

Vec3r RayTracer::specularTermArea(const HitRecord& hit_record, const AreaLight& light, Ray& ray, const Vec3r& point) {
    Material material = scene.materials[hit_record.material_id - 1];

    Vec3r light_direction = light.getLightDirection(hit_record.intersection_point, point);
    real light_distance = light.getLightDistance(light_direction);
    Vec3r wi = normalizeVec3r(light_direction);

    real cos_alpha = std::max((real)0, dotVec3r(light.getNormal(), -wi));        // Angle between normal and wi
    if (cos_alpha < 0) cos_alpha = -cos_alpha;
    Vec3r radiance = light.getRadiance();                                       // Light radiance
    real area = light.getSize() * light.getSize();                              // Light area
    Vec3r irradiance = (radiance * area * cos_alpha) / pow(light_distance,2);

    Vec3r wo = normalizeVec3r(-ray.direction);

    Vec3r h = normalizeVec3r(wi + wo);
    cos_alpha = std::max((real)0, dotVec3r(hit_record.normal, h));
    return material.getSpecularReflectance() * pow(cos_alpha, material.getPhongExponent()) * irradiance;
}

Vec3r RayTracer::specularTermDirection(const HitRecord& hit_record, const DirectionalLight& light, Ray& ray) {
    Material material = scene.materials[hit_record.material_id - 1];

    Vec3r light_direction = -light.getDirection();
    Vec3r wi = normalizeVec3r(light_direction);

    Vec3r irradiance = light.getRadiance();
    Vec3r wo = normalizeVec3r(-ray.direction);

    Vec3r h = normalizeVec3r(wi + wo);
    real cos_alpha = std::max((real)0, dotVec3r(hit_record.normal, h));

    return hit_record.ks * pow(cos_alpha, material.getPhongExponent()) * irradiance;
}

Vec3r RayTracer::specularTermSpot(const HitRecord& hit_record, const SpotLight& light, Ray& ray, Vec3r light_direction, real angle) {
    Material material = scene.materials[hit_record.material_id - 1];

    real light_distance = light.getLightDistance(light_direction);
    Vec3r wi = normalizeVec3r(light_direction);

    real f_angle = light.getFallOffAngle() / 2;
    real c_angle = light.getCoverageAngle() / 2;

    real fall_off_factor = 1;
    if (angle > f_angle) {
        fall_off_factor = pow((cos(toRadians(angle)) - cos(toRadians(c_angle))) / 
                          (cos(toRadians(f_angle)) - cos(toRadians(c_angle))), 4);
    }

    Vec3r irradiance = light.getIntensity() / pow(light_distance,2) * fall_off_factor;
    Vec3r wo = normalizeVec3r(-ray.direction);

    Vec3r h = normalizeVec3r(wi + wo);
    real cos_alpha = std::max((real)0, dotVec3r(hit_record.normal, h));

    return hit_record.ks * pow(cos_alpha, material.getPhongExponent()) * irradiance;
}

bool RayTracer::inShadow(const HitRecord& hit_record, const PointLight& light, Vec3r light_direction, real light_distance) {
    Vec3r wi = normalizeVec3r(light_direction);

    Ray shadow_ray;
    shadow_ray.start_point = hit_record.intersection_point +  hit_record.normal * scene.shadow_ray_epsilon;
    shadow_ray.direction = wi;

    for (auto& plane : scene.planes) {
        Ray localRay = transformRay(shadow_ray, plane.getInverseTransformation());

        Vec3r localPoint = scene.vertex_data[plane.getPointID() - 1];
        real t_local = plane.checkPlaneIntersection(localRay, localPoint);

        if (std::isfinite(t_local)) {
            Vec3r localHit = localRay.start_point + t_local * localRay.direction;
            Vec3r worldHit = applyTransformToPoint(plane.getTransformation(), localHit);

            real t_world = euclideanDistanceVec3r(shadow_ray.start_point, worldHit);

            if (t_world < light_distance + 1e-4)
                return true;
        }
    }

    for (auto& sphere : scene.spheres) {
        Ray localRay = transformRay(shadow_ray, sphere.getInverseTransformation());
        Vec3r center = scene.vertex_data[sphere.getCenterVertexID() - 1];
        real t_local = sphere.checkSphereIntersection(localRay, center);

        if (std::isfinite(t_local)) {
            // Transform hit point to world space to get real distance
            Vec3r localHit = localRay.start_point + t_local * localRay.direction;
            Vec3r worldHit = applyTransformToPoint(sphere.getTransformation(), localHit);

            real t_world = euclideanDistanceVec3r(shadow_ray.start_point, worldHit);
            if (t_world < light_distance)
                return true;
        }
    }

    for (auto& triangle : scene.triangles) {
        Ray localRay = transformRay(shadow_ray, triangle.getInverseTransformation());
        Vec3r a = scene.vertex_data[triangle.getV0ID() - 1];
        Vec3r b = scene.vertex_data[triangle.getV1ID() - 1];
        Vec3r c = scene.vertex_data[triangle.getV2ID() - 1];

        real t_local = triangle.checkTriangleIntersection(localRay, a, b, c, "flat");

        if (std::isfinite(t_local)) {
            Vec3r localHit = localRay.start_point + t_local * localRay.direction;
            Vec3r worldHit = applyTransformToPoint(triangle.getTransformation(), localHit);

            real t_world = euclideanDistanceVec3r(shadow_ray.start_point, worldHit);
            if (t_world < light_distance)
                return true;
        }
    }

    for (auto& mesh : scene.meshes) {
        Mat4r inv_mat = mesh.getInverseTransformation();
        if (mesh.isMotionBlurred()) {
            Vec3r mv = mesh.getMotionVector() * scene.samples[curr_sample_index].psi_5;
            real tx = mv.x, ty = mv.y, tz = mv.z;
            Mat4r motionMatrix =  Mat4r::createTranslation(tx, ty, tz);
            Mat4r transform_mat = mesh.getTransformation();
            transform_mat = motionMatrix * transform_mat;
            inv_mat = transform_mat.inverse();
        }
        Ray localRay = transformRay(shadow_ray, inv_mat);
        HitRecord temp;
        if (mesh.getBVH().intersect(localRay, temp)) {
            Vec3r worldHit = applyTransformToPoint(mesh.getTransformation(), temp.intersection_point);
            real t_world = euclideanDistanceVec3r(shadow_ray.start_point, worldHit);
            if (t_world < light_distance)
                return true;
        }
    }

    for (auto& inst : scene.mesh_instances) {
        Mat4r inv_mat = inst.inv_transformation;
        if (inst.isMotionBlurred()) {
            Vec3r mv = inst.motion_vector * scene.samples[curr_sample_index].psi_5;
            real tx = mv.x, ty = mv.y, tz = mv.z;
            Mat4r motionMatrix =  Mat4r::createTranslation(tx, ty, tz);
            Mat4r transform_mat = inst.transformation;
            transform_mat = motionMatrix * transform_mat;
            inv_mat = transform_mat.inverse();
        }
        Ray localRay = transformRay(shadow_ray, inv_mat);
        HitRecord temp;
        int id = getMeshByID(scene, inst.baseMeshObjectId);
        Mesh& baseMesh = scene.meshes[id];

        if (baseMesh.getBVH().intersect(localRay, temp)) {
            Vec3r worldHit = applyTransformToPoint(inst.transformation, temp.intersection_point);
            real t_world = euclideanDistanceVec3r(shadow_ray.start_point, worldHit);
            if (t_world < light_distance)
                return true;
        }
    }
    return false;
}

Ray RayTracer::transformRay(const Ray& ray, const Mat4r& mat) {
    glm::mat4 gmat = mat.toGlm();

    glm::vec4 o(ray.start_point.x, ray.start_point.y, ray.start_point.z, 1.0);
    glm::vec4 o_new = gmat * o;

    glm::vec4 d(ray.direction.x, ray.direction.y, ray.direction.z, 0.0);
    glm::vec4 d_new = gmat * d;

    Ray transformed;
    transformed.start_point = Vec3r(o_new.x, o_new.y, o_new.z);
    transformed.direction = normalizeVec3r(Vec3r(d_new.x, d_new.y, d_new.z));

    transformed.distance = ray.distance;
    transformed.depth = ray.depth;
    transformed.isIn = ray.isIn;
    transformed.n = ray.n;

    return transformed;
}

void RayTracer::transformHitToWorld(
    const Mat4r& objToWorld,
    const Vec3r& localHit,
    const Vec3r& localNormal,
    Vec3r& worldHit,
    Vec3r& worldNormal
) {
    glm::mat4 gm = objToWorld.toGlm();

    // Transform point (w = 1)
    glm::vec4 p(localHit.x, localHit.y, localHit.z, 1.0);
    glm::vec4 p_world = gm * p;
    worldHit = Vec3r(p_world.x, p_world.y, p_world.z);

    // Transform normal (w = 0) with inverse transpose of upper-left 3x3
    glm::mat3 normalMat = glm::transpose(glm::inverse(glm::mat3(gm)));
    glm::vec3 n_world = normalMat * glm::vec3(localNormal.x, localNormal.y, localNormal.z);
    worldNormal = normalizeVec3r(Vec3r(n_world.x, n_world.y, n_world.z));
}

std::vector<real> RayTracer::precomputeGaussianWeights(int nSamples) {
    std::vector<real> weights(nSamples);
    int grid_dim = sqrt(nSamples); 
    
    real sigma = 1.0 / 6.0; // Change this to affect the blurr
    real pixel_center = 0.5f;
    real total_weight = 0.0f;

    for (int j = 0; j < grid_dim; ++j) {
        for (int i = 0; i < grid_dim; ++i) {
            real sub_x = (i + 0.5f) / grid_dim;
            real sub_y = (j + 0.5f) / grid_dim;

            real dist_x = sub_x - pixel_center;
            real dist_y = sub_y - pixel_center;
            
            real dist_sq = (dist_x * dist_x) + (dist_y * dist_y);

            int index = j * grid_dim + i;
            real w = 1.0 / (2 *  M_PI * sigma * sigma) * std::exp(-dist_sq / (2.0f * sigma * sigma));
            
            weights[index] = w;
            total_weight += w;
        }
    }
    for (int k = 0; k < nSamples; ++k) {
        weights[k] /= total_weight;
    }
    scene.gaussian_weights = weights;

    return weights;
}

void RayTracer::generateRandoms(real sample_length) {
    int total_samples = sample_length * sample_length;

    std::vector<Vec2r> pixel_samples(total_samples);
    std::vector<Vec2r> aperture_samples(total_samples);
    std::vector<Vec2r> area_light_samples(total_samples);
    std::vector<Vec2r> roughness_samples(total_samples);
    std::vector<real> time_samples(total_samples);

    std::mt19937 gRandomGenerator(0);
    std::uniform_real_distribution<> rnd(0, 1);

    int i = 0;
    for (int y = 0; y < sample_length; ++y) {
        for (int x = 0; x < sample_length; ++x) {
            pixel_samples[i].x = (x + rnd(gRandomGenerator)) / sample_length;
            pixel_samples[i].y = (y + rnd(gRandomGenerator)) / sample_length;

            aperture_samples[i].x = (x + rnd(gRandomGenerator)) / sample_length;
            aperture_samples[i].y = (y + rnd(gRandomGenerator)) / sample_length;

            area_light_samples[i].x = (x + rnd(gRandomGenerator)) / sample_length;
            area_light_samples[i].y = (y + rnd(gRandomGenerator)) / sample_length;

            roughness_samples[i].x = (x + rnd(gRandomGenerator)) / sample_length;
            roughness_samples[i].y = (y + rnd(gRandomGenerator)) / sample_length;

            time_samples[i] = (i + rnd(gRandomGenerator)) / total_samples;

            i += 1;
        }
    }

    std::shuffle(aperture_samples.begin(), aperture_samples.end(), gRandomGenerator);
    std::shuffle(area_light_samples.begin(), area_light_samples.end(), gRandomGenerator); 
    std::shuffle(roughness_samples.begin(), roughness_samples.end(), gRandomGenerator);   
    std::shuffle(time_samples.begin(), time_samples.end(), gRandomGenerator);


    scene.samples.resize(total_samples);
    for (int k = 0; k < total_samples; ++k) {
        scene.samples[k].psi_1 = pixel_samples[k].x;
        scene.samples[k].psi_2 = pixel_samples[k].y;

        scene.samples[k].psi_3 = aperture_samples[k].x;
        scene.samples[k].psi_4 = aperture_samples[k].y;

        scene.samples[k].psi_5 = time_samples[k];

        scene.samples[k].psi_6 = area_light_samples[k].x;
        scene.samples[k].psi_7 = area_light_samples[k].y;

        scene.samples[k].psi_8 = roughness_samples[k].x;
        scene.samples[k].psi_9 = roughness_samples[k].y;
    }
}

Vec3r RayTracer::computeEnvironmentColor(Image tex_img, Vec3r d, std::string type) {
    real pi = 3.14159;
    real u,v;
    if (type == "latlong") {
        u = (1 + atan2(d.x, - d.z) / pi) * 0.5;
        v = acos(d.y) / pi;
    }
    else if (type == "probe") {
        real r = 1/pi * (acos(-d.z) / pow(d.x * d.x + d.y * d.y, 0.5));

        u = (r * d.x + 1) * 0.5;
        v = (-r * d.y + 1) * 0.5;
    }

    real tex_w = u * tex_img.width;
    real tex_h = v * tex_img.height;

    return applyBilinearInterpolation(tex_img.data, tex_img.width, tex_img.height, tex_img.channels, tex_w, tex_h);
}

bool RayTracer::updateHitRecord(HitRecord& rec, real t, const Vec3r& point, const Vec3r& normal, int matID) {
    if (t < rec.t && t > 0) { // Assuming 0 is epsilon
        rec.t = t;
        rec.intersection_point = point;
        rec.normal = normal;
        rec.material_id = matID;
        
        if (matID > 0) {
            rec.kd = scene.materials[matID - 1].getDiffuseReflectance();
            rec.ks = scene.materials[matID - 1].getSpecularReflectance();
        }
        return true;
    }
    return false;
}

void RayTracer::applyTextureShading (
    HitRecord& hit_record,
    const std::vector<int>& textureIDs,
    const TextureContext& ctx,
    const Mat4r& invTranspose) 
{
    for (int texID : textureIDs) {
        TextureMap tex_map = scene.texture.getTexMap(texID - 1);

        if (tex_map.type == "image") {
            Image tex_img = scene.texture.getImage(tex_map.image_ID - 1);
            
            Vec3r color = sampleTextureColor(tex_map, tex_img, ctx.u, ctx.v);

            // --- Decal Modes ---
            if (tex_map.decal_mode == "replace_kd") {
                hit_record.kd = color;
            }
            else if (tex_map.decal_mode == "replace_ks") {
                hit_record.ks = color;
            }
            else if (tex_map.decal_mode == "blend_kd") {
                hit_record.kd = (color + hit_record.kd) / 2.0;
            }
            else if (tex_map.decal_mode == "replace_all") {
                hit_record.kd = color;
                hit_record.replaceAll = true;
            }
            else if (tex_map.decal_mode == "replace_normal") {
                Vec3r map_normal = normalizeVec3r(color * 2.0 - 1.0);
                
                Vec3r N = normalizeVec3r(ctx.localNormal);
                Vec3r T = normalizeVec3r(ctx.tangentU);
                Vec3r B = normalizeVec3r(ctx.tangentV);

                Vec3r new_local_N = T * map_normal.x + B * map_normal.y + N * map_normal.z;
                new_local_N = normalizeVec3r(new_local_N);

                hit_record.normal = normalizeVec3r(applyTransformToVector(invTranspose, new_local_N));
            }
            else if (tex_map.decal_mode == "bump_normal") {
                real h = (color.x + color.y + color.z) / 3.0;

                real epsilon_u = 1.0 / tex_img.width;
                real epsilon_v = 1.0 / tex_img.height;

                Vec3r c_u = sampleTextureColor(tex_map, tex_img, ctx.u + epsilon_u, ctx.v);
                real h_u = (c_u.x + c_u.y + c_u.z) / 3.0;

                Vec3r c_v = sampleTextureColor(tex_map, tex_img, ctx.u, ctx.v + epsilon_v);
                real h_v = (c_v.x + c_v.y + c_v.z) / 3.0;

                real dh_du = (h_u - h) * tex_map.bump_factor / epsilon_u;
                real dh_dv = (h_v - h) * tex_map.bump_factor / epsilon_v;

                Vec3r dq_du = ctx.tangentU + dh_du * ctx.localNormal;
                Vec3r dq_dv = ctx.tangentV + dh_dv * ctx.localNormal;
                Vec3r new_local_N = normalizeVec3r(crossVec3r(dq_dv, dq_du));

                hit_record.normal = normalizeVec3r(applyTransformToVector(invTranspose, new_local_N));
            }
        }

        else if (tex_map.type == "perlin") {
            auto computeNoise = [&](Vec3r p) -> real {
                real val = per.noise(p * tex_map.noise_scale, tex_map.noise_conversion);
                for (int oct = 1; oct < tex_map.num_octaves; ++oct) {
                    val += pow(2, -oct) * per.noise(p * pow(2, oct), tex_map.noise_conversion);
                }
                return val;
            };

            real noise = computeNoise(ctx.localPoint);

            if (tex_map.decal_mode == "replace_kd") {
                hit_record.kd = Vec3r(noise, noise, noise);
            }
            else if (tex_map.decal_mode == "blend_kd") {
                hit_record.kd = (Vec3r(noise, noise, noise) + hit_record.kd) / 2.0;
            }
            else if (tex_map.decal_mode == "bump_normal") {
                real eps = 0.001;
                
                real h = noise;
                
                Vec3r p_u = ctx.localPoint + ctx.tangentU * eps;
                real h_u = computeNoise(p_u);
                
                Vec3r p_v = ctx.localPoint + ctx.tangentV * eps;
                real h_v = computeNoise(p_v);

                real dh_du = (h_u - h) * tex_map.bump_factor / eps;
                real dh_dv = (h_v - h) * tex_map.bump_factor / eps;

                Vec3r dq_du = ctx.tangentU + dh_du * ctx.localNormal;
                Vec3r dq_dv = ctx.tangentV + dh_dv * ctx.localNormal;
                Vec3r new_local_N = normalizeVec3r(crossVec3r(dq_dv, dq_du));

                hit_record.normal = normalizeVec3r(applyTransformToVector(invTranspose, new_local_N));
            }
        }
        
        else if (tex_map.type == "checkerboard") {
            real scale = tex_map.scale;
            real offset = tex_map.offset;
            
            bool x = ((int)floor((ctx.localPoint.x + offset) * scale)) % 2 != 0;
            bool y = ((int)floor((ctx.localPoint.y + offset) * scale)) % 2 != 0;
            bool z = ((int)floor((ctx.localPoint.z + offset) * scale)) % 2 != 0;

            if ((x ^ y) ^ z) {
                hit_record.kd = tex_map.white_color;
            } else {
                hit_record.kd = tex_map.black_color;
            }
        }
    }
}

bool RayTracer::processPlaneHit(Ray& ray, Plane& plane, HitRecord& hit_record) {
    Ray localRay = transformRay(ray, plane.getInverseTransformation());
    Vec3r localPoint = scene.vertex_data[plane.getPointID() - 1];

    real t_local = plane.checkPlaneIntersection(localRay, localPoint);

    if (std::isfinite(t_local)) {
        Vec3r localHit = localRay.start_point + t_local * localRay.direction;
        Vec3r localNormal = plane.getNormal();

        Vec3r intersection_point, normal;
        transformHitToWorld(plane.getTransformation(), localHit, localNormal, intersection_point, normal);

        real t_world = euclideanDistanceVec3r(ray.start_point, intersection_point);

        if (updateHitRecord(hit_record, t_world, intersection_point, normal, plane.getMaterialID())) {
            
            if (plane.isTextured()) {
                Vec3r tan_u = normalizeVec3r(localPoint - localHit);
                Vec3r tan_v = normalizeVec3r(crossVec3r(tan_u, localNormal));
                tan_u = normalizeVec3r(crossVec3r(localNormal, tan_v));

                TextureContext ctx;
                ctx.localPoint = localHit;
                ctx.localNormal = localNormal;
                ctx.tangentU = tan_u;
                ctx.tangentV = tan_v;
                ctx.u = -1; // Procedural textures usually, don't need explicit UVs here
                ctx.v = -1;

                Mat4r invTr = plane.getInverseTransformation().transpose();
                applyTextureShading(hit_record, plane.getTextures(), ctx, invTr);
            }
            
            return true; 
        }
    }
    return false; 
}

bool RayTracer::processSphereHit(Ray& ray, Sphere& sphere, HitRecord& hit_record) {
    Ray localRay = transformRay(ray, sphere.getInverseTransformation());
    Vec3r localCenter = scene.vertex_data[sphere.getCenterVertexID() - 1];

    real t_local = sphere.checkSphereIntersection(localRay, localCenter);

    if (std::isfinite(t_local)) {
        Vec3r localHit = localRay.start_point + t_local * localRay.direction;
        Vec3r localNormal = normalizeVec3r(localHit - localCenter);

        Vec3r intersection_point, normal;
        transformHitToWorld(sphere.getTransformation(), localHit, localNormal, intersection_point, normal);

        real t_world = euclideanDistanceVec3r(ray.start_point, intersection_point);

        if (updateHitRecord(hit_record, t_world, intersection_point, normal, sphere.getMaterialID())) {
            
            if (sphere.isTextured()) {
                Vec3r local_p = localHit - localCenter;
                real pi = 2 * acos(0.0); 

                real theta = acos(local_p.y / sphere.getRadius());
                real phi = atan2(local_p.z, local_p.x);

                real tex_u = (-phi + pi) / (2 * pi);
                real tex_v = theta / pi;

                tex_u -= floor(tex_u);
                tex_v -= floor(tex_v);

                Vec3r tan_u = sphere.computeTangentU(local_p, pi);
                Vec3r tan_v = crossVec3r(localNormal, tan_u);
                tan_u = crossVec3r(tan_v, localNormal); 

                TextureContext ctx;
                ctx.localPoint = localHit; 
                ctx.localNormal = localNormal;
                ctx.tangentU = tan_u;
                ctx.tangentV = tan_v;
                ctx.u = tex_u;
                ctx.v = tex_v;

                Mat4r invTr = sphere.getInverseTransformation().transpose();
                applyTextureShading(hit_record, sphere.getTextures(), ctx, invTr);                
            }

            return true; 
        }
    }
    return false;
}

bool RayTracer::processTriangleHit(Ray& ray, Triangle& triangle, HitRecord& hit_record) {
    Ray localRay = transformRay(ray, triangle.getInverseTransformation());

    Vec3r a = scene.vertex_data[triangle.getV0ID() - 1];
    Vec3r b = scene.vertex_data[triangle.getV1ID() - 1];
    Vec3r c = scene.vertex_data[triangle.getV2ID() - 1];

    real t_local = triangle.checkTriangleIntersection(localRay, a, b, c, "flat");

    if (std::isfinite(t_local)) {
        Vec3r localHit = localRay.start_point + t_local * localRay.direction;
        Vec3r localNormal = normalizeVec3r(crossVec3r(b - a, c - a));

        Vec3r intersection_point, normal;
        transformHitToWorld(triangle.getTransformation(), localHit, localNormal, intersection_point, normal);

        real t_world = euclideanDistanceVec3r(ray.start_point, intersection_point);

        if (updateHitRecord(hit_record, t_world, intersection_point, normal, triangle.getMaterialID())) {
            return true;
        }
    }
    return false;
}

bool RayTracer::processMeshHit(Ray& ray, Mesh& mesh, HitRecord& hit_record) {
    Mat4r finalTransform = mesh.getTransformation();
    Mat4r invFinalTransform = mesh.getInverseTransformation();

    if (mesh.isMotionBlurred()) {
        Vec3r mv = mesh.getMotionVector() * scene.samples[curr_sample_index].psi_5;
        Mat4r motionMatrix = Mat4r::createTranslation(mv.x, mv.y, mv.z);
        
        finalTransform = motionMatrix * finalTransform;
        invFinalTransform = finalTransform.inverse();
    }
    
    Ray localRay = transformRay(ray, invFinalTransform);
    HitRecord tempHit;
    
    if (mesh.getBVH().intersect(localRay, tempHit)) {
        
        Vec3r intersection_point, normal;
        transformHitToWorld(finalTransform, tempHit.intersection_point, tempHit.normal, intersection_point, normal);

        real t_world = euclideanDistanceVec3r(ray.start_point, intersection_point);

        if (updateHitRecord(hit_record, t_world, intersection_point, normal, mesh.getMaterialID())) {
            hit_record.traversal_steps = tempHit.traversal_steps;
            if (mesh.isTextured()) {
                Triangle tri = mesh.getTriangles()[localRay.tri_idx];
                
                real beta = localRay.beta;
                real gamma = localRay.gamma;

                Vec2r t0, t1, t2; 
                if (!scene.tex_coord_data.empty()) {
                    t0 = scene.tex_coord_data[tri.getT0ID() - 1];
                    t1 = scene.tex_coord_data[tri.getT1ID() - 1];
                    t2 = scene.tex_coord_data[tri.getT2ID() - 1];
                }

                real u = t0.x + beta * (t1.x - t0.x) + gamma * (t2.x - t0.x);
                real v = t0.y + beta * (t1.y - t0.y) + gamma * (t2.y - t0.y);
                
                u -= floor(u);
                v -= floor(v);

                Vec3r a = scene.vertex_data[tri.getV0ID() - 1];
                Vec3r b = scene.vertex_data[tri.getV1ID() - 1];
                Vec3r c = scene.vertex_data[tri.getV2ID() - 1];

                Vec3r tan_u = tri.computeTangentVectorU(t0, t1, t2, a, b, c);
                Vec3r tan_v = tri.computeTangentVectorV(t0, t1, t2, a, b, c);
                
                TextureContext ctx;
                ctx.localPoint = tempHit.intersection_point; 
                ctx.localNormal = tri.getNormal();
                ctx.tangentU = tan_u;
                ctx.tangentV = tan_v;
                ctx.u = u;
                ctx.v = v;

                applyTextureShading(hit_record, mesh.getTextures(), ctx, invFinalTransform.transpose());  
            }
            
            return true;
        }
    }
    return false;
}

bool RayTracer::processMeshInstanceHit(Ray& ray, MeshInstance& inst, HitRecord& hit_record) {
    if (!rayAABB(ray, inst.aabb_min_world, inst.aabb_max_world, hit_record.t)) {
        return false;
    }

    Mat4r finalTransform = inst.transformation;
    Mat4r invFinalTransform = inst.inv_transformation;

    if (inst.isMotionBlurred()) {
        Vec3r mv = inst.motion_vector * scene.samples[curr_sample_index].psi_5;
        Mat4r motionMatrix = Mat4r::createTranslation(mv.x, mv.y, mv.z);
        
        finalTransform = motionMatrix * finalTransform;
        invFinalTransform = finalTransform.inverse();
    }

    Ray localRay = transformRay(ray, invFinalTransform);

    int meshIdx = inst.baseMeshObjectId;
    int id = getMeshByID(scene, meshIdx); 
    Mesh& baseMesh = scene.meshes[id];

    HitRecord tempHit;
    if (baseMesh.getBVH().intersect(localRay, tempHit)) {
        
        Vec3r intersection_point, normal;
        transformHitToWorld(finalTransform, tempHit.intersection_point, tempHit.normal, intersection_point, normal);

        real t_world = euclideanDistanceVec3r(ray.start_point, intersection_point);
        int effectiveMaterialID = (inst.materialId == -1) ? baseMesh.getMaterialID() : inst.materialId;

        if (updateHitRecord(hit_record, t_world, intersection_point, normal, effectiveMaterialID)) {
            
            return true;
        }
    }
    return false;
}

bool RayTracer::processLightSphereHit(Ray& ray, LightSphere& lSphere, HitRecord& hit_record) {
    Ray localRay = transformRay(ray, lSphere.getInverseTransformation());
    Vec3r localCenter = scene.vertex_data[lSphere.getCenterVertexID() - 1];

    real t_local = lSphere.checkSphereIntersection(localRay, localCenter);

    if (std::isfinite(t_local)) {
        Vec3r localHit = localRay.start_point + t_local * localRay.direction;
        Vec3r localNormal = normalizeVec3r(localHit - localCenter);

        Vec3r intersection_point, normal;
        transformHitToWorld(lSphere.getTransformation(), localHit, localNormal, intersection_point, normal);

        real t_world = euclideanDistanceVec3r(ray.start_point, intersection_point);

        if (updateHitRecord(hit_record, t_world, intersection_point, normal, lSphere.getMaterialID())) {
            
            hit_record.lightObject = true;
            hit_record.object_radiance = lSphere.getRadiance();
            
            return true;
        }
    }
    return false;
}

bool RayTracer::processLightMeshHit(Ray& ray, LightMesh& lMesh, HitRecord& hit_record) {
    bool hitFound = false;
    std::vector<Triangle> tris = lMesh.getTriangles();

    for (auto& tri : tris) {
        Ray localRay = transformRay(ray, tri.getInverseTransformation());

        Vec3r a = scene.vertex_data[tri.getV0ID() - 1];
        Vec3r b = scene.vertex_data[tri.getV1ID() - 1];
        Vec3r c = scene.vertex_data[tri.getV2ID() - 1];

        real t_local = tri.checkTriangleIntersection(localRay, a, b, c, "flat");

        if (std::isfinite(t_local)) {
            Vec3r localHit = localRay.start_point + t_local * localRay.direction;
            Vec3r localNormal = normalizeVec3r(crossVec3r(b - a, c - a));

            Vec3r intersection_point, normal;
            transformHitToWorld(tri.getTransformation(), localHit, localNormal, intersection_point, normal);

            real t_world = euclideanDistanceVec3r(ray.start_point, intersection_point);

            if (updateHitRecord(hit_record, t_world, intersection_point, normal, lMesh.getMaterialID())) {
                hit_record.lightObject = true;
                hit_record.object_radiance = lMesh.getRadiance();
                hit_record.area = tri.getArea();
                
                hitFound = true;
            }
        }
    }
    return hitFound;
}

bool RayTracer::processGridMeshHit(Ray& ray, Mesh& mesh, int triangle_id, HitRecord& hit_record) {
    Mat4r finalTransform = mesh.getTransformation();
    Mat4r invFinalTransform = mesh.getInverseTransformation();

    if (mesh.isMotionBlurred()) {
        Vec3r mv = mesh.getMotionVector() * scene.samples[curr_sample_index].psi_5;
        Mat4r motionMatrix = Mat4r::createTranslation(mv.x, mv.y, mv.z);
        
        finalTransform = motionMatrix * finalTransform;
        invFinalTransform = finalTransform.inverse();
    }
    
    Ray localRay = transformRay(ray, invFinalTransform);

    Triangle& tri = mesh.getTriangles()[triangle_id]; 

    Vec3r a = scene.vertex_data[tri.getV0ID() - 1];
    Vec3r b = scene.vertex_data[tri.getV1ID() - 1];
    Vec3r c = scene.vertex_data[tri.getV2ID() - 1];
    
    real t_local = tri.checkTriangleIntersection(localRay, a, b, c, mesh.getShadingMode());

    if (std::isfinite(t_local)) {
        Vec3r localHitPoint = localRay.start_point + t_local * localRay.direction;
        Vec3r localNormal = tri.getNormal(); 

        Vec3r intersection_point, normal;
        transformHitToWorld(finalTransform, localHitPoint, localNormal, intersection_point, normal);

        real t_world = euclideanDistanceVec3r(ray.start_point, intersection_point);

        if (updateHitRecord(hit_record, t_world, intersection_point, normal, mesh.getMaterialID())) {
            
            if (mesh.isTextured()) {
                real beta = localRay.beta;
                real gamma = localRay.gamma;

                Vec2r t0, t1, t2; 
                if (!scene.tex_coord_data.empty()) {
                    t0 = scene.tex_coord_data[tri.getT0ID() - 1];
                    t1 = scene.tex_coord_data[tri.getT1ID() - 1];
                    t2 = scene.tex_coord_data[tri.getT2ID() - 1];
                }

                real u = t0.x + beta * (t1.x - t0.x) + gamma * (t2.x - t0.x);
                real v = t0.y + beta * (t1.y - t0.y) + gamma * (t2.y - t0.y);
                
                u -= floor(u);
                v -= floor(v);

                Vec3r a = scene.vertex_data[tri.getV0ID() - 1];
                Vec3r b = scene.vertex_data[tri.getV1ID() - 1];
                Vec3r c = scene.vertex_data[tri.getV2ID() - 1];

                Vec3r tan_u = tri.computeTangentVectorU(t0, t1, t2, a, b, c);
                Vec3r tan_v = tri.computeTangentVectorV(t0, t1, t2, a, b, c);
                
                TextureContext ctx;
                ctx.localPoint = localHitPoint; 
                ctx.localNormal = localNormal; 
                ctx.tangentU = tan_u;
                ctx.tangentV = tan_v;
                ctx.u = u;
                ctx.v = v;

                applyTextureShading(hit_record, mesh.getTextures(), ctx, invFinalTransform.transpose());  
            }
            
            return true;
        }
    }
    return false;
}

bool RayTracer::processGridMeshInstanceHit(Ray& ray, MeshInstance& inst, Mesh& baseMesh, int triangle_id, HitRecord& hit_record) {
    Mat4r finalTransform = inst.transformation;
    Mat4r invFinalTransform = inst.inv_transformation;

    if (inst.isMotionBlurred()) {
        Vec3r mv = inst.motion_vector * scene.samples[curr_sample_index].psi_5;
        Mat4r motionMatrix = Mat4r::createTranslation(mv.x, mv.y, mv.z);
        
        finalTransform = motionMatrix * finalTransform;
        invFinalTransform = finalTransform.inverse();
    }
    Ray localRay = transformRay(ray, invFinalTransform);

    std::vector<Triangle>& triangles = baseMesh.getTriangles();
    if (triangle_id < 0 || triangle_id >= triangles.size()) return false;

    Triangle& tri = triangles[triangle_id]; 

    Vec3r a = scene.vertex_data[tri.getV0ID() - 1];
    Vec3r b = scene.vertex_data[tri.getV1ID() - 1];
    Vec3r c = scene.vertex_data[tri.getV2ID() - 1];
    
    real t_local = tri.checkTriangleIntersection(localRay, a, b, c, baseMesh.getShadingMode());

    if (std::isfinite(t_local)) {
        Vec3r localHitPoint = localRay.start_point + t_local * localRay.direction;
        Vec3r localNormal = tri.getNormal(); 

        Vec3r intersection_point, normal;
        transformHitToWorld(finalTransform, localHitPoint, localNormal, intersection_point, normal);

        real t_world = euclideanDistanceVec3r(ray.start_point, intersection_point);

        int effectiveMaterialID = (inst.materialId == -1) ? baseMesh.getMaterialID() : inst.materialId;

        if (updateHitRecord(hit_record, t_world, intersection_point, normal, effectiveMaterialID)) {
            if (baseMesh.isTextured()) {
                real beta = localRay.beta;
                real gamma = localRay.gamma;

                Vec2r t0, t1, t2; 
                if (!scene.tex_coord_data.empty()) {
                    t0 = scene.tex_coord_data[tri.getT0ID() - 1];
                    t1 = scene.tex_coord_data[tri.getT1ID() - 1];
                    t2 = scene.tex_coord_data[tri.getT2ID() - 1];
                }

                real u = t0.x + beta * (t1.x - t0.x) + gamma * (t2.x - t0.x);
                real v = t0.y + beta * (t1.y - t0.y) + gamma * (t2.y - t0.y);
                
                u -= floor(u);
                v -= floor(v);

                Vec3r tan_a = scene.vertex_data[tri.getV0ID() - 1];
                Vec3r tan_b = scene.vertex_data[tri.getV1ID() - 1];
                Vec3r tan_c = scene.vertex_data[tri.getV2ID() - 1];

                Vec3r tan_u = tri.computeTangentVectorU(t0, t1, t2, tan_a, tan_b, tan_c);
                Vec3r tan_v = tri.computeTangentVectorV(t0, t1, t2, tan_a, tan_b, tan_c);
                
                TextureContext ctx;
                ctx.localPoint = localHitPoint; 
                ctx.localNormal = localNormal; 
                ctx.tangentU = tan_u;
                ctx.tangentV = tan_v;
                ctx.u = u;
                ctx.v = v;

                applyTextureShading(hit_record, baseMesh.getTextures(), ctx, invFinalTransform.transpose());  
            }
            
            return true;
        }
    }
    return false;
}

Vec3r hsv2rgb(real h, real s, real v) {
    real c = v * s;
    real x = c * (1.0f - std::abs(std::fmod(h / 60.0f, 2.0f) - 1.0f));
    real m = v - c;

    real r = 0, g = 0, b = 0;
    if (h >= 0 && h < 60) {
        r = c; g = x; b = 0;
    } else if (h >= 60 && h < 120) {
        r = x; g = c; b = 0;
    } else if (h >= 120 && h < 180) {
        r = 0; g = c; b = x;
    } else if (h >= 180 && h < 240) {
        r = 0; g = x; b = c;
    } else if (h >= 240 && h < 300) {
        r = x; g = 0; b = c;
    } else {
        r = c; g = 0; b = x;
    }

    return Vec3r((r + m) * 255.0f, (g + m) * 255.0f, (b + m) * 255.0f);
}

// Vec3r RayTracer::computeHeatMapColor(int cost) {
//     real max_cost = 200.0f; 
    
//     real t = (real)cost / max_cost;
//     if (t > 1.0f) t = 1.0f;

//     real hue = (1.0f - t) * 240.0f;
    
//     return hsv2rgb(hue, 1.0f, 1.0f);
// }

Vec3r RayTracer::computeHeatMapColor(int cost) {
    real max_cost = 200.0f;
    real t = (real)cost / max_cost;
    if (t > 1.0f) t = 1.0f;

    real r = 0, g = 0, b = 0;

    if (t < 0.5f) {
        real local_t = t * 2.0f;
        b = 1.0f - local_t;
        g = local_t;
    } else {
        real local_t = (t - 0.5f) * 2.0f;
        g = 1.0f - local_t;
        r = local_t;
    }

    return Vec3r(r * 255, g * 255, b * 255);
}



