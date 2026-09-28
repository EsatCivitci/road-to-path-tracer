/*

#include "Scene.h"
#include "Parser.h"
#include "RayTracer.h"
#include "BVH.h"
#include "TransformUtils.h"

#include <iostream>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <cstdlib>

namespace fs = std::filesystem;

void renderScene(const std::string &inputPath, const std::string &outputPath) {
    // --- Timer for the whole program ---
    auto totalStartTime = std::chrono::high_resolution_clock::now();

    Scene myScene;
    Parser myParser;
    myParser.parseScene(inputPath, myScene);
    myParser.checkParsedScene(myScene);
    std::cout << "Parsing Done!!" << std::endl;

    std::cout << "Building BVH!!" << std::endl;
    for (auto &mesh : myScene.meshes)
        mesh.buildBVH(myScene);

    for (auto &mi : myScene.mesh_instances) {
        mi.transformation = computeInstanceTransformRecursive(mi, myScene);
        mi.inv_transformation = mi.transformation.inverse();

        int meshID = getMeshByID(myScene, mi.baseMeshObjectId);
        Mesh &baseMesh = myScene.meshes[meshID];
        const BVHNode &node = baseMesh.getBVH().getBVHNodes()[0];
        computeWorldAABB(node.aabb_min, node.aabb_max, mi.transformation,
                         mi.aabb_min_world, mi.aabb_max_world);
    }

    RayTracer myRayTracer(myScene);
    myRayTracer.startRayTracer();

    auto totalEndTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> totalElapsed = totalEndTime - totalStartTime;

    std::cout << "Rendered " << inputPath << " in "
              << totalElapsed.count() << " seconds.\n";
}

int main() {
    std::string inputFolder = "inputs/tunnel_of_doom/";
    std::string outputFolder = "my_outputs/tunnel_of_doom/";

    fs::create_directories(outputFolder);

    int frameIndex = 0;
    for (const auto &entry : fs::directory_iterator(inputFolder)) {
        if (entry.path().extension() == ".json") {
            std::ostringstream ss;
            ss << outputFolder << "frame_" << std::setw(4)
               << std::setfill('0') << frameIndex++ << ".png";

            std::cout << "\n=== Rendering " << entry.path() << " ===\n";
            renderScene(entry.path().string(), ss.str());
        }
    }

    return 0;

}

*/

#include "Scene.h"
#include "Parser.h"
#include "RayTracer.h"
#include "BVH.h"
#include "TransformUtils.h"

#include <iostream>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <cstdlib>

namespace fs = std::filesystem;


int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <scene_file.json>" << std::endl;
        return 1;
    }

    // --- Timer for the whole program ---
    auto totalStartTime = std::chrono::high_resolution_clock::now();

    // ==========================================
    // 1. Preprocessing Stages
    // ==========================================
    
    // --- 1a. Parsing ---
    auto parseStartTime = std::chrono::high_resolution_clock::now();
    
    Scene myScene;
    Parser myParser;
    myParser.parseScene(argv[1], myScene);
    // myParser.checkParsedScene(myScene);
    std::cout << "Parsing Done!!" << std::endl;
    
    auto parseEndTime = std::chrono::high_resolution_clock::now();

    // --- 1b. KD-Tree Build ---
    std::cout << "Building KD-Tree!!" << std::endl;
    myScene.kd_tree.build(myScene, SplitMethod::ObjectMedian);
    
    auto kdEndTime = std::chrono::high_resolution_clock::now();

    // --- 1c. Grid Build ---
    std::cout << "Constructing Grid Structure!!" << std::endl;
    myScene.grid.buildGrid(myScene);
    
    auto gridEndTime = std::chrono::high_resolution_clock::now();

    // --- 1d. BVH Build ---
    std::cout << "Building BVH!!" << std::endl;
    for (auto& mesh : myScene.meshes) {
        mesh.buildBVH(myScene);
    }
    
    auto bvhEndTime = std::chrono::high_resolution_clock::now();

    // --- 1e. Instance Processing ---
    std::cout << "Computing Mesh Instance AABBs!!\n";
    for (auto& mi : myScene.mesh_instances) {
        mi.transformation = computeInstanceTransformRecursive(mi, myScene);
        mi.inv_transformation = mi.transformation.inverse();

        int meshID = getMeshByID(myScene, mi.baseMeshObjectId);
        Mesh& baseMesh = myScene.meshes[meshID];
        const BVHNode& node = baseMesh.getBVH().getBVHNodes()[0];
        
        computeWorldAABB(node.aabb_min, node.aabb_max, mi.transformation, mi.aabb_min_world, mi.aabb_max_world);
    }
    
    auto instanceEndTime = std::chrono::high_resolution_clock::now();


    // ==========================================
    // 2. Ray Tracing (Render)
    // ==========================================
    std::cout << "Start Ray Tracer!!" << std::endl;
    auto rayTraceStartTime = std::chrono::high_resolution_clock::now();
    
    RayTracer myRayTracer(myScene);
    myRayTracer.startRayTracer();

    auto rayTraceEndTime = std::chrono::high_resolution_clock::now();

    // --- End Total Timer ---
    auto totalEndTime = std::chrono::high_resolution_clock::now();


    // ==========================================
    // Calculate Durations
    // ==========================================
    std::chrono::duration<double> parseElapsed    = parseEndTime - parseStartTime;
    std::chrono::duration<double> kdElapsed       = kdEndTime - parseEndTime;
    std::chrono::duration<double> gridElapsed     = gridEndTime - kdEndTime;
    std::chrono::duration<double> bvhElapsed      = bvhEndTime - gridEndTime;
    std::chrono::duration<double> instanceElapsed = instanceEndTime - bvhEndTime;
    
    // Total Preprocessing time is the sum of the above or (instanceEndTime - parseStartTime)
    std::chrono::duration<double> totalPreprocess = instanceEndTime - parseStartTime; 
    
    std::chrono::duration<double> rayTraceElapsed = rayTraceEndTime - rayTraceStartTime;
    std::chrono::duration<double> totalElapsed    = totalEndTime - totalStartTime;

    // ==========================================
    // Print Results
    // ==========================================
    std::cout << "\n=========================================\n";
    std::cout << "TIMING BREAKDOWN\n";
    std::cout << "=========================================\n";
    std::cout << "PREPROCESSING:\n";
    std::cout << "  - Parsing:             " << parseElapsed.count()    << " s\n";
    std::cout << "  - KD-Tree Build:       " << kdElapsed.count()       << " s\n";
    std::cout << "  - Grid Construction:   " << gridElapsed.count()     << " s\n";
    std::cout << "  - BVH Build (Meshes):  " << bvhElapsed.count()      << " s\n";
    std::cout << "  ---------------------------------------\n";
    std::cout << "  Total Preprocessing:   " << totalPreprocess.count() << " s\n\n";
    
    std::cout << "RENDERING:\n";
    std::cout << "  Ray Tracing:           " << rayTraceElapsed.count() << " s\n";
    std::cout << "-----------------------------------------\n";
    std::cout << "TOTAL PROGRAM TIME:      " << totalElapsed.count()    << " s\n";
    std::cout << "=========================================\n";

    return 0;
}
