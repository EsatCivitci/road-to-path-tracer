#include "Parser.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdint>
#include <utility>
#include <filesystem> 
#include "json.hpp"
#include "Utils.h"
#include "Transformation.h"
#include "TransformUtils.h"
#include "BVH.h"

namespace fs = std::filesystem;

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define TINYEXR_IMPLEMENTATION
#include "tinyexr.h"

using json = nlohmann::json;

Parser::Parser () {

}

Parser::~Parser () {

}

Vec3r Parser::parseVec3r(const std::string& s) {
    std::stringstream ss(s);
    Vec3r vec;
    ss >> vec.x >> vec.y >> vec.z;
    return vec;
}

Vec2r Parser::parseVec2r(const std::string& s) {
    std::stringstream ss(s);
    Vec2r vec;
    ss >> vec.x >> vec.y;
    return vec;
}

Vec2i Parser::parseVec2i(const std::string& s) {
    std::stringstream ss(s);
    Vec2i vec;
    ss >> vec.x >> vec.y;
    return vec;
}

std::vector<real> Parser::parseNearPlane(const std::string& s) {
    std::stringstream ss(s);
    std::vector<real> plane;
    real val;
    while (ss >> val) {
        plane.push_back(val);
    }
    return plane;
}

std::tuple<real, real, real, real> parseRotation(const std::string& s) {
    std::istringstream iss(s);
    real angle, x, y, z;
    iss >> angle >> x >> y >> z;
    return {angle, x, y, z};
}

std::string getPath(const std::string& filename) {
    std::filesystem::path p(filename);
    return p.parent_path().string() + "/"; // Returns "brdf/inputs"
}

static void parseTransformList(const json& obj, std::vector<std::string>& outList)
{
    if (!obj.contains("Transformations")) return;

    std::istringstream iss(obj["Transformations"].get<std::string>());
    std::string token;
    while (iss >> token)
        outList.push_back(token);
}

void Parser::parseScene(const std::string& filename, Scene& scene) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return;
    }

    json j;
    file >> j;

    if (!j.contains("Scene")) return;
    auto& jsonScene = j["Scene"];

    std::string folder_path = getPath(filename);
    
    bool zeroBaseIndexing = false;

    // Parse Scene settings
    if (jsonScene.contains("MaxRecursionDepth")) scene.max_recursion_depth = std::stoi(jsonScene["MaxRecursionDepth"].get<std::string>());
    if (jsonScene.contains("BackgroundColor")) scene.background_color = parseVec3r(jsonScene["BackgroundColor"].get<std::string>());
    if (jsonScene.contains("ShadowRayEpsilon")) scene.shadow_ray_epsilon = std::stof(jsonScene["ShadowRayEpsilon"].get<std::string>());
    if (jsonScene.contains("IntersectionTestEpsilon")) scene.intersection_test_epsilon = std::stof(jsonScene["IntersectionTestEpsilon"].get<std::string>());
    if (jsonScene.contains("ZeroBasedIndexing")) {
        std::string zbi = jsonScene["ZeroBasedIndexing"].get<std::string>();
        if (zbi == "true") zeroBaseIndexing = true;
    }

    // --- PARSE TRANSFORMATIONS ---
    if (jsonScene.contains("Transformations")) {
        const auto& tr = jsonScene["Transformations"];

        // --- PARSE TRANSLATION ---
        if (tr.contains("Translation")) {
            const auto& t = tr["Translation"];

            auto parseTranslation = [&](const json& obj) {
                if (!obj.contains("_id") || !obj.contains("_data")) {
                    std::cerr << "[ERROR] Invalid Translation object in JSON\n";
                    return;
                }

                std::string id = "t" + obj["_id"].get<std::string>();  // <-- Prefix added
                Vec3r v = parseVec3r(obj["_data"].get<std::string>());

                scene.transformations[id] = Transformation::Translation(v.x, v.y, v.z);
            };

            if (t.is_array()) {
                for (const auto& obj : t) parseTranslation(obj);
            } else if (t.is_object()) {
                parseTranslation(t);
            }
        }

        // --- PARSE SCALING ---
        if (tr.contains("Scaling")) {
            const auto& s = tr["Scaling"];

            auto parseScaling = [&](const json& obj) {
                if (!obj.contains("_id") || !obj.contains("_data")) {
                    std::cerr << "[ERROR] Invalid Scaling object in JSON\n";
                    return;
                }

                std::string id = "s" + obj["_id"].get<std::string>();  // <-- Prefix added
                Vec3r scale = parseVec3r(obj["_data"].get<std::string>());

                scene.transformations[id] = Transformation::Scaling(scale.x, scale.y, scale.z);
            };

            if (s.is_array()) {
                for (const auto& obj : s) parseScaling(obj);
            } else if (s.is_object()) {
                parseScaling(s);
            }
        }

        // --- PARSE ROTATION ---
        if (tr.contains("Rotation")) {
            const auto& r = tr["Rotation"];

            auto parseRotationObj = [&](const json& obj) {
                if (!obj.contains("_id") || !obj.contains("_data")) {
                    std::cerr << "[ERROR] Invalid Rotation object in JSON\n";
                    return;
                }

                std::string id = "r" + obj["_id"].get<std::string>();  // <-- Prefix added
                auto [angle, x, y, z] = parseRotation(obj["_data"].get<std::string>());

                scene.transformations[id] = Transformation::Rotation(angle, x, y, z);
            };

            if (r.is_array()) {
                for (const auto& obj : r) parseRotationObj(obj);
            } else if (r.is_object()) {
                parseRotationObj(r);
            }
        }

        // --- PARSE COMPOSITE MATRICES ---
        if (tr.contains("Composite")) {
            const auto& c = tr["Composite"];

            auto parseComposite = [&](const json& obj) {
                if (!obj.contains("_id") || !obj.contains("_data")) {
                    std::cerr << "[ERROR] Invalid Composite object in JSON\n";
                    return;
                }

                std::string id = "c" + obj["_id"].get<std::string>();  // <-- Prefix added
                std::string dataString = obj["_data"].get<std::string>();

                std::istringstream iss(dataString);
                real values[16];
                for (int i = 0; i < 16; i++) {
                    if (!(iss >> values[i])) {
                        std::cerr << "[ERROR] Composite matrix for id=" << id
                                << " does not contain 16 numeric values!\n";
                        return;
                    }
                }

                scene.transformations[id] = Transformation::Composite(values);
            };

            if (c.is_array()) {
                for (const auto& obj : c) parseComposite(obj);
            } else if (c.is_object()) {
                parseComposite(c);
            }
        }
    }

    // --- PARSE CAMERA ---
    if (jsonScene.contains("Cameras") && jsonScene["Cameras"].contains("Camera")) {
        auto parseCameraObject = [&](const json& camJson) {
            Camera cam;
            ToneMap tm;
            if (camJson.contains("_id")) {
                int id = std::stoi(camJson["_id"].get<std::string>());
                if (zeroBaseIndexing) id++;
                cam.setID(id);
            }
            if (camJson.contains("Position")){ 
                Vec3r position = parseVec3r(camJson["Position"].get<std::string>());
                cam.setPosition(position);
            }
            if (camJson.contains("Up")) {
                Vec3r up = parseVec3r(camJson["Up"].get<std::string>());
                cam.setUp(normalizeVec3r(up));
            }
            if (camJson.contains("NearDistance")) { 
                real near_distance = std::stof(camJson["NearDistance"].get<std::string>());
                cam.setNearDistance(near_distance);
            }
            if (camJson.contains("ImageResolution")) { 
                Vec2i image_resolution = parseVec2i(camJson["ImageResolution"].get<std::string>());
                cam.setImageResolution(image_resolution);
            }
            if (camJson.contains("ImageName")) { 
                std::string image_name = camJson["ImageName"].get<std::string>();
                cam.setImageName(image_name);
            }
            if (camJson.contains("ApertureSize")) { 
                real aperture_size = std::stof(camJson["ApertureSize"].get<std::string>());
                cam.setApertureSize(aperture_size);
            }
            if (camJson.contains("NumSamples")) { 
                int num_samples = std::stoi(camJson["NumSamples"].get<std::string>());
                cam.setNumSamples(num_samples);
            }
            if (camJson.contains("FocusDistance")) { 
                real focus_distance = std::stof(camJson["FocusDistance"].get<std::string>());
                cam.setFocusDistance(focus_distance);
            }
            if (camJson.contains("Tonemap")) {
                ToneMap tone_map;
                auto parseToneMapObject = [&](const json& tmJson) {
                    TM tm;
                    if (tmJson.contains("TMO")) {
                        tm.TMO = tmJson["TMO"].get<std::string>();
                    }
                    if (tmJson.contains("TMOOptions")) {
                        Vec2r temp = parseVec2r(tmJson["TMOOptions"].get<std::string>());
                        tm.key = temp.x;
                        tm.burn_out = temp.y;
                    }
                    if (tmJson.contains("Saturation")) {
                        tm.saturation = std::stof(tmJson["Saturation"].get<std::string>());
                    }
                    if (tmJson.contains("Gamma")) {
                        tm.gamma = std::stof(tmJson["Gamma"].get<std::string>());
                    }
                    if (tmJson.contains("Extension")) {
                        tm.extension = tmJson["Extension"].get<std::string>();
                    }
                    tone_map.addTM(tm);
                };
                const auto& toneMapValue = camJson["Tonemap"];
                if (toneMapValue.is_array()) {
                    for (const auto& tmJson : toneMapValue) parseToneMapObject(tmJson);
                } else if (toneMapValue.is_object()) {
                    parseToneMapObject(toneMapValue);
                }
                cam.setTM(tone_map);
            }
            if (camJson.contains("Renderer")) {
                std::string renderer = camJson["Renderer"].get<std::string>();
                cam.setRenderer(renderer);
            }

            parseTransformList(camJson, cam.getTransformIds());


            // --- TYPE 1: LookAt camera ---
            if (camJson.contains("_type") && camJson["_type"].get<std::string>() == "lookAt") {
                if (camJson.contains("GazePoint")) {
                    Vec3r gazePoint = parseVec3r(camJson["GazePoint"].get<std::string>());
                    Vec3r gaze = normalizeVec3r(gazePoint - cam.getPosition());
                    cam.setGaze(gaze);
                }
                if (camJson.contains("Gaze")) {
                    Vec3r gaze = parseVec3r(camJson["Gaze"].get<std::string>());
                    cam.setGaze(gaze);
                }
                if (camJson.contains("FovY")) {
                    real fovy_deg = std::stof(camJson["FovY"].get<std::string>());
                    real fovy_rad = fovy_deg * M_PI / 180.0;

                    int resX = cam.getImageResolution().x;
                    int resY = cam.getImageResolution().y;
                    real aspect = static_cast<real>(resX) / static_cast<real>(resY);

                    real top = cam.getNearDistance() * std::tan(fovy_rad / 2.0);
                    real bottom = -top;
                    real right = top * aspect;
                    real left = -right;

                    std::vector<real> near_plane = {left, right, bottom, top};
                    cam.setNearPlane(near_plane);
                } else {
                    std::cerr << "Warning: lookAt camera missing FovY field.\n";
                }
            }
            // --- TYPE 2: Legacy camera with explicit NearPlane ---
            else {
                if (camJson.contains("Gaze")) {
                    Vec3r gaze = parseVec3r(camJson["Gaze"].get<std::string>());
                    cam.setGaze(gaze);
                }
                if (camJson.contains("NearPlane")) {
                    std::vector<real> near_plane = parseNearPlane(camJson["NearPlane"].get<std::string>());
                    cam.setNearPlane(near_plane);
                }
            }
            // after parsing all fields
            if (!cam.getTransformIds().empty()) {
                Mat4r total = Mat4r::identity();
                cam.isTransformed = true;
                // Compose all transformations
                for (const std::string& tid : cam.getTransformIds()) {
                    auto it = scene.transformations.find(tid);
                    if (it != scene.transformations.end()) {
                        total = it->second.getMatrix() * total;
                    } else {
                        std::cerr << "[Warning] Camera " << cam.getID()
                                << " refers to missing transformation ID '" << tid << "'\n";
                    }
                }
                // Apply to camera attributes
                cam.setPosition(applyTransformToPoint(total, cam.getPosition()));
                cam.setGaze(applyTransformToVectorSpecial(total, cam.getGaze()));
                cam.setUp(applyTransformToVectorSpecial(total, cam.getUp()));
            }

            scene.cameras.push_back(cam);
        };
        const auto& cameraValue = jsonScene["Cameras"]["Camera"];
        if (cameraValue.is_array()) {
            for (const auto& camJson : cameraValue) parseCameraObject(camJson);
        } else if (cameraValue.is_object()) {
            parseCameraObject(cameraValue);
        }
    }

    // --- PARSE LIGHTS ---
    if (jsonScene.contains("Lights")) {
        if (jsonScene["Lights"].contains("AmbientLight")) {
             scene.ambient_light = parseVec3r(jsonScene["Lights"]["AmbientLight"].get<std::string>());
        }
        
        if (jsonScene["Lights"].contains("PointLight")) {
            auto parsePointLightObject = [&](const json& lightJson) {
                PointLight pl;
                if (lightJson.contains("_id")) {
                    int id = std::stoi(lightJson["_id"].get<std::string>());
                    if (zeroBaseIndexing) id++;
                    pl.setID(id);
                }
                if (lightJson.contains("Position")) {
                    Vec3r position = parseVec3r(lightJson["Position"].get<std::string>());
                    pl.setPosition(position);
                }
                if (lightJson.contains("Intensity")) {
                    Vec3r intensity = parseVec3r(lightJson["Intensity"].get<std::string>());
                    pl.setIntensity(intensity);
                }
                parseTransformList(lightJson, pl.getTransformIds());

                // After parsing all PointLight fields
                if (!pl.getTransformIds().empty()) {
                    Mat4r total = Mat4r::identity();

                    // Compose all transformations
                    for (const std::string& tid : pl.getTransformIds()) {
                        auto it = scene.transformations.find(tid);
                        if (it != scene.transformations.end()) {
                            const Transformation& t = it->second;
                            // Only apply rotations or translations
                            if (t.typeName() == "Translation" || t.typeName() == "Rotation") {
                                total = t.getMatrix() * total;
                            } else {
                                std::cerr << "[Info] Ignoring non-positional transform '"
                                        << tid << "' for PointLight " << pl.getID() << "\n";
                            }
                        } else {
                            std::cerr << "[Warning] PointLight " << pl.getID()
                                    << " refers to missing transformation ID '" << tid << "'\n";
                        }
                    }

                    // Apply final transformation only to position (as point → w = 1)
                    pl.setPosition(applyTransformToPoint(total, pl.getPosition()));
                }
                scene.point_lights.push_back(pl);
            };
            const auto& pointLightValue = jsonScene["Lights"]["PointLight"];
            if (pointLightValue.is_array()) {
                for (const auto& lightJson : pointLightValue) parsePointLightObject(lightJson);
            } else if (pointLightValue.is_object()) {
                parsePointLightObject(pointLightValue);
            }

        }
        
        if (jsonScene["Lights"].contains("AreaLight")) {
            auto parseAreaLightObject = [&](const json& lightJson) {
                AreaLight al;

                if (lightJson.contains("_id")) {
                    int id = std::stoi(lightJson["_id"].get<std::string>());
                    if (zeroBaseIndexing) id++;
                    al.setID(id);
                }
                if (lightJson.contains("Position")) {
                    Vec3r pos = parseVec3r(lightJson["Position"].get<std::string>());
                    al.setPosition(pos);
                }
                if (lightJson.contains("Normal")) {
                    Vec3r norm = parseVec3r(lightJson["Normal"].get<std::string>());
                    norm = normalizeVec3r(norm);

                    Vec3r u;
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
                    Vec3r v = normalizeVec3r(crossVec3r(u, norm));

                    al.setNormal(norm);
                    al.setU(u);
                    al.setv(v);
                }
                if (lightJson.contains("Size")) {
                    real size = std::stof(lightJson["Size"].get<std::string>());
                    al.setSize(size);
                }
                if (lightJson.contains("Radiance")) {
                    Vec3r rad = parseVec3r(lightJson["Radiance"].get<std::string>());
                    al.setRadiance(rad);
                }                
                if (lightJson.contains("Intensity")) {
                    Vec3r rad = parseVec3r(lightJson["Intensity"].get<std::string>());
                    al.setRadiance(rad);
                }
                scene.area_lights.push_back(al);
            };
            const auto& areaLightValue = jsonScene["Lights"]["AreaLight"];
            if (areaLightValue.is_array()) {
                for (const auto& lightJson : areaLightValue) parseAreaLightObject(lightJson);
            } else if (areaLightValue.is_object()) {
                parseAreaLightObject(areaLightValue);
            }
        }
    
        if (jsonScene["Lights"].contains("DirectionalLight")) {
            auto parseDirLightObject = [&](const json& lightJson) {
                DirectionalLight dl;
                if (lightJson.contains("_id")) {
                    int id = std::stoi(lightJson["_id"].get<std::string>());
                    if (zeroBaseIndexing) id++;
                    dl.setID(id);
                }
                if (lightJson.contains("Direction")) {
                    Vec3r dir = parseVec3r(lightJson["Direction"].get<std::string>());
                    dl.setDirection(dir);
                }
                if (lightJson.contains("Radiance")) {
                    Vec3r rad = parseVec3r(lightJson["Radiance"].get<std::string>());
                    dl.setRadiance(rad);
                }
                scene.dir_lights.push_back(dl);
            };
            const auto& dirLightValue = jsonScene["Lights"]["DirectionalLight"];
            if (dirLightValue.is_array()) {
                for (const auto& lightJson : dirLightValue) parseDirLightObject(lightJson);
            } else if (dirLightValue.is_object()) {
                parseDirLightObject(dirLightValue);
            }
        }

        if (jsonScene["Lights"].contains("SpotLight")) {
            auto parseSpotLightObject = [&](const json& lightJson) {
                SpotLight sl;
                if (lightJson.contains("_id")) {
                    int id = std::stoi(lightJson["_id"].get<std::string>());
                    if (zeroBaseIndexing) id++;
                    sl.setID(id);
                }
                if (lightJson.contains("Position")) {
                    Vec3r pos = parseVec3r(lightJson["Position"].get<std::string>());
                    sl.setPosition(pos);
                }
                if (lightJson.contains("Direction")) {
                    Vec3r dir = parseVec3r(lightJson["Direction"].get<std::string>());
                    sl.setDirection(dir);
                }
                if (lightJson.contains("Intensity")) {
                    Vec3r intensity = parseVec3r(lightJson["Intensity"].get<std::string>());
                    sl.setIntensity(intensity);
                }
                if (lightJson.contains("CoverageAngle")) {
                    real c_ang = std::stof(lightJson["CoverageAngle"].get<std::string>());
                    sl.setCoverageAngle(c_ang);
                }
                if (lightJson.contains("FalloffAngle")) {
                    real f_ang = std::stoi(lightJson["FalloffAngle"].get<std::string>());
                    sl.setFallOffAngle(f_ang);
                }
                scene.spot_lights.push_back(sl);
            };
            const auto& spotLightValue = jsonScene["Lights"]["SpotLight"];
            if (spotLightValue.is_array()) {
                for (const auto& lightJson : spotLightValue) parseSpotLightObject(lightJson);
            } else if (spotLightValue.is_object()) {
                parseSpotLightObject(spotLightValue);
            }
        }
    
    }

    if (jsonScene.contains("BRDFs")) {
       
        if (jsonScene["BRDFs"].contains("OriginalBlinnPhong")) {
            auto parseOBPObject = [&](const json& obpJson) {
                BRDF brdf;
                brdf.type = "OriginalBlinnPhong";
                std::cout << "OriginalBlinnPhong" << std::endl;
                if (obpJson.contains("_id")) {
                    brdf.ID = std::stoi(obpJson["_id"].get<std::string>());
                    if (zeroBaseIndexing) brdf.ID++;
                    std::cout << brdf.ID << std::endl;
                }
                if (obpJson.contains("_normalized")) {
                    std::string val = obpJson["_normalized"].get<std::string>();
                    if (val == "true") brdf.normalized = true;
                    else brdf.normalized = false;
                }
                if (obpJson.contains("Exponent")) {
                    brdf.exponent = std::stof(obpJson["Exponent"].get<std::string>());
                    std::cout << brdf.exponent << std::endl;
                }
                scene.BRDFs.push_back(brdf);
            };

            const auto& obpValue = jsonScene["BRDFs"]["OriginalBlinnPhong"];
            if (obpValue.is_array()) {
                for (const auto& obpJson : obpValue) parseOBPObject(obpJson);
            } else if (obpValue.is_object()) {
                parseOBPObject(obpValue);
            }
        }
        
        if (jsonScene["BRDFs"].contains("OriginalPhong")) {
            auto parseOBPObject = [&](const json& obpJson) {
                BRDF brdf;
                brdf.type = "OriginalPhong";
                std::cout << "OriginalPhong" << std::endl;
                if (obpJson.contains("_id")) {
                    brdf.ID = std::stoi(obpJson["_id"].get<std::string>());
                    if (zeroBaseIndexing) brdf.ID++;
                    std::cout << brdf.ID << std::endl;
                }
                if (obpJson.contains("_normalized")) {
                    std::string val = obpJson["_normalized"].get<std::string>();
                    if (val == "true") brdf.normalized = true;
                    else brdf.normalized = false;
                }
                if (obpJson.contains("Exponent")) {
                    brdf.exponent = std::stof(obpJson["Exponent"].get<std::string>());
                    std::cout << brdf.exponent << std::endl;
                }
                scene.BRDFs.push_back(brdf);
            };

            const auto& obpValue = jsonScene["BRDFs"]["OriginalPhong"];
            if (obpValue.is_array()) {
                for (const auto& obpJson : obpValue) parseOBPObject(obpJson);
            } else if (obpValue.is_object()) {
                parseOBPObject(obpValue);
            }
        }        
        
        if (jsonScene["BRDFs"].contains("ModifiedBlinnPhong")) {
            auto parseOBPObject = [&](const json& obpJson) {
                BRDF brdf;
                brdf.type = "ModifiedBlinnPhong";
                std::cout << "ModifiedBlinnPhong" << std::endl;
                if (obpJson.contains("_id")) {
                    brdf.ID = std::stoi(obpJson["_id"].get<std::string>());
                    if (zeroBaseIndexing) brdf.ID++;
                    std::cout << brdf.ID << std::endl;
                }
                if (obpJson.contains("_normalized")) {
                    std::string val = obpJson["_normalized"].get<std::string>();
                    if (val == "true") brdf.normalized = true;
                    else brdf.normalized = false;
                }
                if (obpJson.contains("Exponent")) {
                    brdf.exponent = std::stof(obpJson["Exponent"].get<std::string>());
                    std::cout << brdf.exponent << std::endl;
                }
                scene.BRDFs.push_back(brdf);
            };

            const auto& obpValue = jsonScene["BRDFs"]["ModifiedBlinnPhong"];
            if (obpValue.is_array()) {
                for (const auto& obpJson : obpValue) parseOBPObject(obpJson);
            } else if (obpValue.is_object()) {
                parseOBPObject(obpValue);
            }
        }        
        
        if (jsonScene["BRDFs"].contains("ModifiedPhong")) {
            auto parseOBPObject = [&](const json& obpJson) {
                BRDF brdf;
                brdf.type = "ModifiedPhong";
                std::cout << "ModifiedPhong" << std::endl;
                if (obpJson.contains("_id")) {
                    brdf.ID = std::stoi(obpJson["_id"].get<std::string>());
                    if (zeroBaseIndexing) brdf.ID++;
                    std::cout << brdf.ID << std::endl;
                }
                if (obpJson.contains("_normalized")) {
                    std::string val = obpJson["_normalized"].get<std::string>();
                    if (val == "true") brdf.normalized = true;
                    else brdf.normalized = false;
                }
                if (obpJson.contains("Exponent")) {
                    brdf.exponent = std::stof(obpJson["Exponent"].get<std::string>());
                    std::cout << brdf.exponent << std::endl;
                }
                scene.BRDFs.push_back(brdf);
            };

            const auto& obpValue = jsonScene["BRDFs"]["ModifiedPhong"];
            if (obpValue.is_array()) {
                for (const auto& obpJson : obpValue) parseOBPObject(obpJson);
            } else if (obpValue.is_object()) {
                parseOBPObject(obpValue);
            }
        }

        if (jsonScene["BRDFs"].contains("TorranceSparrow")) {
            auto parseOBPObject = [&](const json& obpJson) {
                BRDF brdf;
                brdf.type = "TorranceSparrow";
                std::cout << "TorranceSparrow" << std::endl;
                if (obpJson.contains("_id")) {
                    brdf.ID = std::stoi(obpJson["_id"].get<std::string>());
                    if (zeroBaseIndexing) brdf.ID++;
                    std::cout << brdf.ID << std::endl;
                }
                if (obpJson.contains("_normalized")) {
                    std::string val = obpJson["_normalized"].get<std::string>();
                    if (val == "true") brdf.normalized = true;
                    else brdf.normalized = false;
                }
                if (obpJson.contains("Exponent")) {
                    brdf.exponent = std::stof(obpJson["Exponent"].get<std::string>());
                    std::cout << brdf.exponent << std::endl;
                }
                if (obpJson.contains("_kdfresnel")) {
                    std::string val = obpJson["_kdfresnel"].get<std::string>();
                    if (val == "true") brdf.kd_fresnel = true;
                    else brdf.kd_fresnel = false;
                    std::cout << brdf.kd_fresnel << std::endl;
                }
                scene.BRDFs.push_back(brdf);
            };

            const auto& obpValue = jsonScene["BRDFs"]["TorranceSparrow"];
            if (obpValue.is_array()) {
                for (const auto& obpJson : obpValue) parseOBPObject(obpJson);
            } else if (obpValue.is_object()) {
                parseOBPObject(obpValue);
            }
        }
    }

    // --- PARSE MATERIALS ---
    if (jsonScene.contains("Materials") && jsonScene["Materials"].contains("Material")) {
        auto parseMaterialObject = [&](const json& matJson) {
            Material m;
            bool degamma = false;
            if (matJson.contains("_degamma")) {
                std::string s = matJson["_degamma"].get<std::string>();
                if (s == "true") degamma = true;
            }
            if (matJson.contains("_id")){
                 int id = std::stoi(matJson["_id"].get<std::string>());
                 if (zeroBaseIndexing) id++;
                 m.setID(id);
            }
            if (matJson.contains("_type")){
                std::string type = matJson["_type"].get<std::string>();
                m.setType(type);
            }
            if (matJson.contains("AmbientReflectance")){
                Vec3r ambient_reflectance = parseVec3r(matJson["AmbientReflectance"].get<std::string>());
                m.setAmbientReflectance(ambient_reflectance);
            }
            if (matJson.contains("DiffuseReflectance")){
                Vec3r diffuse_reflectance = parseVec3r(matJson["DiffuseReflectance"].get<std::string>());
                if (degamma) {
                    diffuse_reflectance.x = pow(diffuse_reflectance.x, 2.2);
                    diffuse_reflectance.y = pow(diffuse_reflectance.y, 2.2);
                    diffuse_reflectance.z = pow(diffuse_reflectance.z, 2.2);
                } 
                m.setDiffuseReflectance(diffuse_reflectance);
            }
            if (matJson.contains("SpecularReflectance")){
                Vec3r specular_reflectance = parseVec3r(matJson["SpecularReflectance"].get<std::string>());
                if (degamma) {
                    specular_reflectance.x = pow(specular_reflectance.x, 2.2);
                    specular_reflectance.y = pow(specular_reflectance.y, 2.2);
                    specular_reflectance.z = pow(specular_reflectance.z, 2.2);
                } 
                m.setSpecularReflectance(specular_reflectance);
            }
            if (matJson.contains("PhongExponent")){
                real phong_exponent = std::stof(matJson["PhongExponent"].get<std::string>());
                m.setPhongExponent(phong_exponent);
            }
            if (matJson.contains("MirrorReflectance")){
                Vec3r mirror_reflectance = parseVec3r(matJson["MirrorReflectance"].get<std::string>());
                m.setMirrorReflectance(mirror_reflectance);
            }
            if (matJson.contains("RefractionIndex")){
                real refraction_index = std::stof(matJson["RefractionIndex"].get<std::string>());
                if (zeroBaseIndexing) refraction_index++;
                m.setRefractionIndex(refraction_index);
            }
            if (matJson.contains("AbsorptionIndex")){
               real absorption_index = std::stof(matJson["AbsorptionIndex"].get<std::string>());
               if (zeroBaseIndexing) absorption_index++;
               m.setAbsorptionIndex(absorption_index);
            }
            if (matJson.contains("AbsorptionCoefficient")){
                Vec3r absorption_coefficient = parseVec3r(matJson["AbsorptionCoefficient"].get<std::string>());
                m.setAbsorptionCoefficient(absorption_coefficient);
            }
            if (matJson.contains("Roughness")) {
                real rough = std::stof(matJson["Roughness"].get<std::string>());
                m.setRoughness(rough);
            }
            if (matJson.contains("_BRDF")) {
                size_t BRDF_ID = std::stoi(matJson["_BRDF"].get<std::string>());
                m.setBRDFID(BRDF_ID);
            }
            scene.materials.push_back(m);
        };
        const auto& materialValue = jsonScene["Materials"]["Material"];
        if (materialValue.is_array()) {
            for (const auto& matJson : materialValue) parseMaterialObject(matJson);
        } else if (materialValue.is_object()) {
            parseMaterialObject(materialValue);
        }
    }

    // --- PARSE TEXTURES ---
    if (jsonScene.contains("Textures")) {
        Texture t;
        if (jsonScene["Textures"].contains("Images") && jsonScene["Textures"]["Images"].contains("Image")) {
            auto parseTextureImage = [&](const json& texImgJson) {
                Image img;

                if (texImgJson.contains("_data")) {
                    // std::string img_path = "inputs/" + texImgJson["_data"].get<std::string>();
                    std::string img_path = folder_path + texImgJson["_data"].get<std::string>();
                    img.path_name = img_path;
                    fs::path p(img_path);

                    std::string ext = p.extension().string();

                    if (ext == ".png" || ext == ".jpg" || ext == ".jpeg") {
                        img.data = stbi_loadf(img_path.c_str(), &img.width, &img.height, &img.channels, STBI_rgb);
                    } 
                    else if (ext == ".exr") {
                        std::cout << "Detected HDR image." << std::endl;

                        float* raw_exr_data = nullptr;
                        const char* err = nullptr;

                        int ret = LoadEXR(&img.data, &img.width, &img.height, img_path.c_str(), &err);
                        img.channels = 4; 
                    }
                }
                if (texImgJson.contains("_id")) {
                    img.ID = stoi(texImgJson["_id"].get<std::string>());
                    if (zeroBaseIndexing) img.ID++;
                }
                t.addImage(img);
            };

            const auto& texImageValue = jsonScene["Textures"]["Images"]["Image"];
            if (texImageValue.is_array()) {
                for (const auto& texImgJson : texImageValue) parseTextureImage(texImgJson);
            } else if (texImageValue.is_object()) {
                 parseTextureImage(texImageValue);
            }
        }
        if (jsonScene["Textures"].contains("TextureMap")) {
            auto parseTextureMap = [&](const json& texMapJson) {
                TextureMap tm;

                if (texMapJson.contains("_id")) {
                    tm.ID = std::stoi(texMapJson["_id"].get<std::string>());
                }
                if (texMapJson.contains("_type")) {
                    tm.type = texMapJson["_type"].get<std::string>();
                }
                if (texMapJson.contains("ImageId")) {
                    tm.image_ID = std::stoi(texMapJson["ImageId"].get<std::string>());
                }
                if (texMapJson.contains("DecalMode")) {
                    tm.decal_mode = texMapJson["DecalMode"].get<std::string>();
                    if (tm.decal_mode == "replace_background") {
                        t.setBackgroundTextureID(tm.ID);
                    }
                }
                if (texMapJson.contains("Interpolation")) {
                    tm.interpolation = texMapJson["Interpolation"].get<std::string>();
                }
                if (texMapJson.contains("BumpFactor")) {
                    tm.bump_factor = std::stof(texMapJson["BumpFactor"].get<std::string>());
                }
                if (texMapJson.contains("Normalizer")) {
                    tm.normalizer = std::stof(texMapJson["Normalizer"].get<std::string>());
                }
                if (texMapJson.contains("NoiseConversion")) {
                    tm.noise_conversion = texMapJson["NoiseConversion"].get<std::string>();
                }
                if (texMapJson.contains("NoiseScale")) {
                    tm.noise_scale = std::stof(texMapJson["NoiseScale"].get<std::string>());
                }
                if (texMapJson.contains("NumOctaves")) {
                    tm.num_octaves = std::stoi(texMapJson["NumOctaves"].get<std::string>());
                }
                if (texMapJson.contains("BlackColor")) {
                    tm.black_color = parseVec3r(texMapJson["BlackColor"].get<std::string>());
                }
                if (texMapJson.contains("WhiteColor")) {
                    tm.black_color = parseVec3r(texMapJson["WhiteColor"].get<std::string>());
                }
                if (texMapJson.contains("Scale")) {
                    tm.scale = std::stof(texMapJson["Scale"].get<std::string>());
                }
                if (texMapJson.contains("Offset")) {
                    tm.offset = std::stof(texMapJson["Offset"].get<std::string>());
                }

                t.addTexMap(tm);

            };

            const auto& texMapValue = jsonScene["Textures"]["TextureMap"];
            if (texMapValue.is_array()) {
                for (const auto& texMapJson : texMapValue) parseTextureMap(texMapJson);
            } else if (texMapValue.is_object()) {
                parseTextureMap(texMapValue);
            }
        }
        scene.texture = t;
    }

    if (jsonScene.contains("Lights")) {
            if (jsonScene["Lights"].contains("SphericalDirectionalLight")) {
            const json& lightJson = jsonScene["Lights"]["SphericalDirectionalLight"];
            EnvironmentLight el;
            if (lightJson.contains("_id")){
                int id = std::stoi(lightJson["_id"].get<std::string>());
                if (zeroBaseIndexing) id++;
                el.setID(id);
            }
            if (lightJson.contains("_type")){
                std::string type = lightJson["_type"].get<std::string>();
                el.setType(type);
            }
            if (lightJson.contains("ImageId")){
                int image_id = std::stoi(lightJson["ImageId"].get<std::string>());
                if (zeroBaseIndexing) image_id++;
                el.setImageID(image_id);
            }
            if (lightJson.contains("Sampler")){
                std::string sampler = lightJson["Sampler"].get<std::string>();
                el.setSampler(sampler);
                std::cout << el.getSampler() << std::endl;
            }
            scene.env_lights.push_back(el);
        }
    }

    // --- PARSE VERTEX DATA ---
    if (jsonScene.contains("VertexData") && jsonScene["VertexData"].contains("_data")) {
        std::stringstream vertexStream(jsonScene["VertexData"]["_data"].get<std::string>());
        Vec3r vertex;
        while (vertexStream >> vertex.x >> vertex.y >> vertex.z) {
            scene.vertex_data.push_back(vertex);
        }
    }

    // --- PARSE TEXCOORD DATA ---
    if (jsonScene.contains("TexCoordData") && jsonScene["TexCoordData"].contains("_data")) {
        std::stringstream texCoordStream(jsonScene["TexCoordData"]["_data"].get<std::string>());
        Vec2r tex_coord;
        while (texCoordStream >> tex_coord.x >> tex_coord.y) {
            scene.tex_coord_data.push_back(tex_coord);
        }
    }

    // --- PARSE OBJECTS ---
    if (jsonScene.contains("Objects")) {
        auto& jsonObjects = jsonScene["Objects"];

        if (jsonObjects.contains("Plane")) {
            auto parsePlaneObject = [&](const json& planeJson) {
                Plane p;
                if (planeJson.contains("_id")) {
                    int id = std::stoi(planeJson["_id"].get<std::string>());
                    if (zeroBaseIndexing) id++;
                    p.setID(id);
                }
                if (planeJson.contains("Point")) {
                    int point_id = std::stoi(planeJson["Point"].get<std::string>());
                    if (zeroBaseIndexing) point_id++;
                    p.setPointID(point_id);
                }
                if (planeJson.contains("Material")) {
                    int material_id = std::stoi(planeJson["Material"].get<std::string>());
                    if (zeroBaseIndexing) material_id++;
                    p.setMaterialID(material_id);
                }
                if (planeJson.contains("Normal")) {
                    Vec3r normal = parseVec3r(planeJson["Normal"].get<std::string>());
                    p.setNormal(normal);
                }

                if (planeJson.contains("Textures")) {
                    int id;
                    std::stringstream texIDStream(planeJson["Textures"].get<std::string>());
                    while (texIDStream >> id) {
                        if (zeroBaseIndexing) id++;
                        p.addTextures(id);
                    }
                }

                parseTransformList(planeJson, p.getTransformIds());

                if (!p.getTransformIds().empty()) {
                    Mat4r total = Mat4r::identity();
                    for (const std::string& tid : p.getTransformIds()) {
                        auto it = scene.transformations.find(tid);
                        if (it != scene.transformations.end()) {
                            total = it->second.getMatrix() * total;
                        } else {
                            std::cerr << "[Warning] Triangle refers to missing transformation ID '"
                                    << tid << "'\n";
                        }
                    }
                    p.setTransformation(total);
                }

                scene.planes.push_back(p);

            };
            const auto& planeValue = jsonObjects["Plane"];
            if (planeValue.is_array()) {
                for (const auto& planeJson : planeValue) parsePlaneObject(planeJson);
            } else if (planeValue.is_object()) {
                parsePlaneObject(planeValue);
            }
        }

        if (jsonObjects.contains("Triangle")) {
            auto parseTriangleObject = [&](const json& triJson) {
                Triangle t;
                if (triJson.contains("_id")) {
                    int id = std::stoi(triJson["_id"].get<std::string>()); 
                    if (zeroBaseIndexing) id++;
                    t.setID(id);
                }

                if (triJson.contains("Material")) {
                    int material_id = std::stoi(triJson["Material"].get<std::string>());
                    if (zeroBaseIndexing) material_id++;
                    t.setMaterialID(material_id);
                }
                if (triJson.contains("Indices")) {
                    std::stringstream indices(triJson["Indices"].get<std::string>());
                    int v0, v1, v2;
                    indices >> v0 >> v1 >> v2;
                    if (zeroBaseIndexing) {
                        v0++;v1++;v2++;
                    }
                    t.setVertexIDs(v0, v1, v2);
                }
                Vec3r v0 = scene.vertex_data[t.getV0ID()-1];
                Vec3r v1 = scene.vertex_data[t.getV1ID()-1];
                Vec3r v2 = scene.vertex_data[t.getV2ID()-1];
                t.setNormal(normalizeVec3r(crossVec3r(v1-v0, v2-v0)));
                t.setCentroid((v0+v1+v2)/3.0);

                parseTransformList(triJson, t.getTransformIds());

                if (!t.getTransformIds().empty()) {
                    Mat4r total = Mat4r::identity();
                    for (const std::string& tid : t.getTransformIds()) {
                        auto it = scene.transformations.find(tid);
                        if (it != scene.transformations.end()) {
                            total = it->second.getMatrix() * total;
                        } else {
                            std::cerr << "[Warning] Triangle refers to missing transformation ID '"
                                    << tid << "'\n";
                        }
                    }
                    t.setTransformation(total);
                }
                scene.triangles.push_back(t);
            };
            const auto& triangleValue = jsonObjects["Triangle"];
            if (triangleValue.is_array()) {
                for (const auto& triJson : triangleValue) parseTriangleObject(triJson);
            } else if (triangleValue.is_object()) {
                parseTriangleObject(triangleValue);
            }
        }
        
        if (jsonObjects.contains("Sphere")) {
            auto parseSphereObject = [&](const json& sphJson) {
                Sphere s;
                if (sphJson.contains("_id")) {
                    int id = std::stoi(sphJson["_id"].get<std::string>());
                    if (zeroBaseIndexing) id++;
                    s.setID(id);
                }
                if (sphJson.contains("Material")) {
                    int material_id = std::stoi(sphJson["Material"].get<std::string>());
                    if (zeroBaseIndexing) material_id++;
                    s.setMaterialID(material_id);
                }
                if (sphJson.contains("Center")) {
                    int center_vertex_id = std::stoi(sphJson["Center"].get<std::string>());
                    if (zeroBaseIndexing) center_vertex_id++;
                    s.setCenterVertexID(center_vertex_id);
                }
                if (sphJson.contains("Radius")) {
                    real radius = std::stof(sphJson["Radius"].get<std::string>());
                    s.setRadius(radius);
                }
                parseTransformList(sphJson, s.getTransformIds());

                if (!s.getTransformIds().empty()) {
                    Mat4r total = Mat4r::identity();
                    for (const std::string& tid : s.getTransformIds()) {
                        auto it = scene.transformations.find(tid);
                        if (it != scene.transformations.end()) {
                            total = it->second.getMatrix() * total;
                        } else {
                            std::cerr << "[Warning] Sphere refers to missing transformation ID '" << tid << "'\n";
                        }
                    }
                    s.setTransformation(total);
                }

                if (sphJson.contains("Textures")) {
                    int id;
                    std::stringstream texIDStream(sphJson["Textures"].get<std::string>());
                    while (texIDStream >> id) {
                        s.addTextures(id);
                    }
                }

                scene.spheres.push_back(s);
            };
            const auto& sphereValue = jsonObjects["Sphere"];
            if (sphereValue.is_array()) {
                for (const auto& sphJson : sphereValue) parseSphereObject(sphJson);
            } else if (sphereValue.is_object()) {
                parseSphereObject(sphereValue);
            }
        }

        if (jsonObjects.contains("LightSphere")) {
            std::cout << "LightSphere" << std::endl;
            auto parseLightSphereObject = [&](const json& lsJson) {
                LightSphere ls;
                if (lsJson.contains("_id")) {
                    int id = std::stoi(lsJson["_id"].get<std::string>());
                    if (zeroBaseIndexing) id++;
                    ls.setID(id);
                }
                if (lsJson.contains("Material")) {
                    int id = std::stoi(lsJson["Material"].get<std::string>());
                    if (zeroBaseIndexing) id++;
                    ls.setMaterialID(id);
                }
                if (lsJson.contains("Center")) {
                    int id = std::stoi(lsJson["Center"].get<std::string>());
                    if (zeroBaseIndexing) id++;
                    ls.setCenterVertexID(id);
                }
                if (lsJson.contains("Radius")) {
                    real r = std::stof(lsJson["Radius"].get<std::string>());
                    ls.setRadius(r);
                }
                if (lsJson.contains("Radiance")) {
                    Vec3r rad = parseVec3r(lsJson["Radiance"].get<std::string>());
                    ls.setRadiance(rad);
                }
                parseTransformList(lsJson, ls.getTransformIds());

                if (!ls.getTransformIds().empty()) {
                    Mat4r total = Mat4r::identity();
                    for (const std::string& tid : ls.getTransformIds()) {
                        auto it = scene.transformations.find(tid);
                        if (it != scene.transformations.end()) {
                            total = it->second.getMatrix() * total;
                        } else {
                            std::cerr << "[Warning] Sphere refers to missing transformation ID '" << tid << "'\n";
                        }
                    }
                    ls.setTransformation(total);
                }
                scene.light_spheres.push_back(ls);
            };
            
            const auto& lsValue = jsonObjects["LightSphere"];
            if (lsValue.is_array()) {
                for (const auto& lsJson : lsValue) parseLightSphereObject(lsJson);
            } else if (lsValue.is_object()) {
                parseLightSphereObject(lsValue);
            }
        }

        if (jsonObjects.contains("Mesh")) { //|| jsonObjects.contains("LightMesh")) {
            static int mesh_triangle_id_counter = 0;
            auto parseMeshObject = [&](const json& meshJson) {
                bool smooth_shading = false;
                Mesh m;
                if (!meshJson.contains("_id") || !meshJson.contains("Material") || !meshJson.contains("Faces")) return;
                int id = std::stoi(meshJson["_id"].get<std::string>());
                if (zeroBaseIndexing) id++;
                m.setID(id);

                if (meshJson.contains("_shadingMode")) {
                     std::string shading_mode = "smooth";
                     m.setShadingMode("smooth");
                     smooth_shading = true;
                }

                if (meshJson.contains("MotionBlur")) {
                    Vec3r mb = parseVec3r(meshJson["MotionBlur"].get<std::string>());
                    m.setMotionVector(mb);
                    printVec3r(m.getMotionVector(), "MotionVector");
                }

                if (meshJson.contains("Textures")) {
                    int id;
                    std::stringstream texIDStream(meshJson["Textures"].get<std::string>());
                    while (texIDStream >> id) {
                        if (zeroBaseIndexing) id++;
                        m.addTextures(id);
                    }
                }

                int material_id = std::stoi(meshJson["Material"].get<std::string>());
                if (zeroBaseIndexing) material_id++;
                m.setMaterialID(material_id);

                const auto& faces = meshJson["Faces"];

                if (faces.contains("_plyFile")) {
                    std::string sceneDir = folder_path + "ply/";
                    std::string plyRelative = faces["_plyFile"].get<std::string>();
                    std::string plyPath = sceneDir + plyRelative;

                    std::vector<Vec3r> plyVerts;     // vertex positions
                    std::vector<Vec3r> plyNormals;   // optional vertex normals
                    std::vector<Vec3i> plyTris;      // triangle indices (0-based)
                    std::vector<Vec2r> plyTex;

                    if (loadPlyMesh(plyPath, plyVerts, plyTris, plyNormals, plyTex)) {
                        int vertexOffset = static_cast<int>(scene.vertex_data.size());
                        int texOffset = static_cast<int>(scene.tex_coord_data.size());
                        scene.vertex_data.insert(scene.vertex_data.end(), plyVerts.begin(), plyVerts.end());
                        scene.tex_coord_data.insert(scene.tex_coord_data.end(), plyTex.begin(), plyTex.end());
                        // ---- Create triangles from PLY ----
                        for (const auto& tri : plyTris) {
                            Triangle t;
                            int id = mesh_triangle_id_counter++;
                            int material_id = m.getMaterialID();

                            // +1 because Scene uses 1-based vertex indices
                            int v0_id = vertexOffset + tri.x + 1;
                            int v1_id = vertexOffset + tri.y + 1;
                            int v2_id = vertexOffset + tri.z + 1;

                            int t0_id = texOffset + tri.x + 1;
                            int t1_id = texOffset + tri.y + 1;
                            int t2_id = texOffset + tri.z + 1;

                            t.setID(id);
                            t.setMaterialID(material_id);
                            t.setVertexIDs(v0_id, v1_id, v2_id);
                            t.setTextureIDs(t0_id, t1_id, t2_id);

                            // Compute and assign flat-shaded face normal
                            Vec3r v0 = scene.vertex_data[v0_id - 1];
                            Vec3r v1 = scene.vertex_data[v1_id - 1];
                            Vec3r v2 = scene.vertex_data[v2_id - 1];
                            t.setNormal(normalizeVec3r(crossVec3r(v1 - v0, v2 - v0)));
                            t.setCentroid((v0+v1+v2)/3.0);

                            m.addTriangle(t);
                        }

                        std::cout << "Mesh " << m.getID() << " loaded from PLY: "
                                << plyVerts.size() << " vertices, "
                                << plyTris.size() << " triangles.\n";

                        // ---- Case A: PLY contains per-vertex normals ----
                        if (!plyNormals.empty()) {
                            // --- THIS IS THE FIX ---
                            
                            // Loop through the TRIANGLES (T times)
                            for (auto& tri : m.getTriangles()) {
                                
                                // 1. Get the global vertex indices for this triangle
                                int v0_global = tri.getV0ID() - 1;
                                int v1_global = tri.getV1ID() - 1;
                                int v2_global = tri.getV2ID() - 1;

                                // 2. Convert them to local indices for the plyNormals array
                                int v0_local = v0_global - vertexOffset;
                                int v1_local = v1_global - vertexOffset;
                                int v2_local = v2_global - vertexOffset;

                                // 3. (Safety Check) Make sure the indices are valid
                                if (v0_local >= 0 && v0_local < plyNormals.size() &&
                                    v1_local >= 0 && v1_local < plyNormals.size() &&
                                    v2_local >= 0 && v2_local < plyNormals.size()) 
                                {
                                    // 4. Set the normals using a direct O(1) lookup
                                    tri.setV0Normal(normalizeVec3r(plyNormals[v0_local]));
                                    tri.setV1Normal(normalizeVec3r(plyNormals[v1_local]));
                                    tri.setV2Normal(normalizeVec3r(plyNormals[v2_local]));
                                }
                                // else: handle error, maybe the vertexOffset is wrong?
                            }

                            // --- END FIX ---

                            std::cout << "Using vertex normals from PLY file (stored per triangle).\n";
                        }

                        // ---- Case B: No normals in file, but smooth shading requested ----
                        else if (smooth_shading) {
                            computeVertexNormals(scene.vertex_data, m.getTriangles());
                            std::cout << "Computed vertex normals (stored per triangle, smooth shading).\n";
                        }

                    }
                }

                else {
                    if (!faces.contains("_data")) return;
                    
                    int vertex_offset = 0;
                    int texture_offset = 0;
                    if (faces.contains("_vertexOffset")) {
                        vertex_offset = std::stoi(faces["_vertexOffset"].get<std::string>());
                        std::cout << "Vertex Offset: " << vertex_offset << std::endl;
                    }
                    if (faces.contains("_textureOffset")) {
                        texture_offset = std::stoi(faces["_textureOffset"].get<std::string>());
                        std::cout << "Texture Offset: " << texture_offset << std::endl;
                    }
                    std::string faces_data = faces["_data"].get<std::string>();
                    std::stringstream ss(faces_data);
                    int v0_idx, v1_idx, v2_idx;
                    while (ss >> v0_idx >> v1_idx >> v2_idx) {
                        Triangle t;
                        int id = mesh_triangle_id_counter++;
                        int material_id = m.getMaterialID();
                        int v0_id = v0_idx + vertex_offset;
                        int v1_id = v1_idx + vertex_offset;
                        int v2_id = v2_idx + vertex_offset;

                        int t0_id = v0_idx + texture_offset;
                        int t1_id = v1_idx + texture_offset;
                        int t2_id = v2_idx + texture_offset;

                        t.setID(id);
                        t.setMaterialID(material_id);
                        t.setVertexIDs(v0_id, v1_id, v2_id);
                        t.setTextureIDs(t0_id, t1_id, t2_id);

                        Vec3r v0 = scene.vertex_data[t.getV0ID()-1];
                        Vec3r v1 = scene.vertex_data[t.getV1ID()-1];
                        Vec3r v2 = scene.vertex_data[t.getV2ID()-1];
                        t.setNormal(normalizeVec3r(crossVec3r(v1-v0, v2-v0)));
                        t.setCentroid((v0+v1+v2)/3.0);
                        
                        m.addTriangle(t);
                    }

                    if (smooth_shading) {
                        // Compute vertex normals for this mesh using scene vertices
                        computeVertexNormals(scene.vertex_data, m.getTriangles());
                    }
                }

                parseTransformList(meshJson, m.getTransformIds());

                // --- Compose total transformation ---
                if (!m.getTransformIds().empty()) {
                    Mat4r total = Mat4r::identity();
                    for (const std::string& tid : m.getTransformIds()) {
                        auto it = scene.transformations.find(tid);
                        if (it != scene.transformations.end()) {
                            total = it->second.getMatrix() * total;
                        } else {
                            std::cerr << "[Warning] Mesh refers to missing transformation ID '"
                                    << tid << "'\n";
                        }
                    }
                    m.setTransformation(total);
                }

                scene.meshes.push_back(std::move(m));
            };
            
            if (jsonObjects.contains("Mesh")) {
                const auto& meshValue = jsonObjects["Mesh"];
                if (meshValue.is_array()) {
                    for (const auto& meshJson : meshValue) parseMeshObject(meshJson);
                } else if (meshValue.is_object()) {
                    parseMeshObject(meshValue);
                }
            }
        }

        if (jsonObjects.contains("LightMesh")) {
            std::cout << "LightMesh" << std::endl;
            LightMesh lm;
            auto parseLightMeshObject = [&](const json& lmJson) {
                if (lmJson.contains("_id")) {
                    int id = std::stoi(lmJson["_id"].get<std::string>());
                    if (zeroBaseIndexing) id++;
                    lm.setID(id);
                    std::cout << lm.getID() << std::endl;
                }
                if (lmJson.contains("Material")) {
                    int id = std::stoi(lmJson["Material"].get<std::string>());
                    if (zeroBaseIndexing) id++;
                    lm.setMaterialID(id);
                    std::cout << lm.getMaterialID() << std::endl;
                }
                if (lmJson.contains("Radiance")) {
                    Vec3r rad = parseVec3r(lmJson["Radiance"].get<std::string>());
                    lm.setRadiance(rad);
                    printVec3r(lm.getRadiance(), "");
                }
                int id = 0;
                const auto& faces = lmJson["Faces"];
                if (faces.contains("_type") || true) {
                    if (!faces.contains("_data")) return;
                    
                    int vertex_offset = 0;
                    int texture_offset = 0;
                    if (faces.contains("_vertexOffset")) {
                        vertex_offset = std::stoi(faces["_vertexOffset"].get<std::string>());
                    }

                    std::string faces_data = faces["_data"].get<std::string>();
                    std::stringstream ss(faces_data);
                    int v0_idx, v1_idx, v2_idx;
                    while (ss >> v0_idx >> v1_idx >> v2_idx) {
                        Triangle t;
                        // int id = mesh_triangle_id_counter++;
                        int material_id = lm.getMaterialID();
                        int v0_id = v0_idx + vertex_offset;
                        int v1_id = v1_idx + vertex_offset;
                        int v2_id = v2_idx + vertex_offset;

                        int t0_id = v0_idx + texture_offset;
                        int t1_id = v1_idx + texture_offset;
                        int t2_id = v2_idx + texture_offset;

                        t.setID(id++);
                        t.setMaterialID(material_id);
                        t.setVertexIDs(v0_id, v1_id, v2_id);
                        t.setTextureIDs(t0_id, t1_id, t2_id);

                        Vec3r v0 = scene.vertex_data[t.getV0ID()-1];
                        Vec3r v1 = scene.vertex_data[t.getV1ID()-1];
                        Vec3r v2 = scene.vertex_data[t.getV2ID()-1];
                        t.setNormal(normalizeVec3r(crossVec3r(v1-v0, v2-v0)));
                        printVec3r(t.getNormal(), "norm");
                        Vec3r e1 = v1 - v0;
                        Vec3r e2 = v2 - v0;
                        real area = 0.5 * lengthSquared(crossVec3r(e1, e2));
                        t.setArea(area);
                        t.setCentroid((v0+v1+v2)/3.0);
                        
                        lm.addTriangle(t);
                    }
                }
                parseTransformList(lmJson, lm.getTransformIds());

                if (!lm.getTransformIds().empty()) {
                    Mat4r total = Mat4r::identity();
                    for (const std::string& tid : lm.getTransformIds()) {
                        auto it = scene.transformations.find(tid);
                        if (it != scene.transformations.end()) {
                            total = it->second.getMatrix() * total;
                        } else {
                            std::cerr << "[Warning] Sphere refers to missing transformation ID '" << tid << "'\n";
                        }
                    }
                    lm.setTransformation(total);
                }
                printMat4r(lm.getTransformation());
                scene.light_meshes.push_back(lm);
            };
            
            const auto& lmValue = jsonObjects["LightMesh"];
            if (lmValue.is_array()) {
                for (const auto& lmJson : lmValue) parseLightMeshObject(lmJson);
            } else if (lmValue.is_object()) {
                parseLightMeshObject(lmValue);
            }
        }

        // --- PARSE MESH INSTANCES ---
        if (jsonObjects.contains("MeshInstance")) {
            const auto& miJson = jsonObjects["MeshInstance"];
            auto parseMeshInstance = [&](const json& obj) {
                MeshInstance mi;

                if (obj.contains("_id"))
                    mi.id = std::stoi(obj["_id"].get<std::string>());

                if (obj.contains("_baseMeshId"))
                    mi.baseMeshId = std::stoi(obj["_baseMeshId"].get<std::string>());
                
                if (obj.contains("Material"))
                    mi.materialId = std::stoi(obj["Material"].get<std::string>());

                if (zeroBaseIndexing){
                    mi.id++;
                    mi.baseMeshId++;
                    mi.materialId++;
                }

                if (obj.contains("_resetTransform"))
                    mi.resetTransform = (obj["_resetTransform"].get<std::string>() == "true");

                if (obj.contains("Transformations")) {
                    std::istringstream iss(obj["Transformations"].get<std::string>());
                    std::string token;
                    while (iss >> token) mi.transformIds.push_back(token);
                }

                if (obj.contains("MotionBlur")) {
                    Vec3r mb = parseVec3r(obj["MotionBlur"].get<std::string>());
                    mi.motion_vector = mb;
                    printVec3r(mi.motion_vector, "MotionVector");
                }

                // --- Compose instance transformation matrix ---
                Mat4r instanceTransform =
                    composeTransformFromIds(mi.transformIds, scene.transformations, "MeshInstance " + std::to_string(mi.id));

                if (mi.baseMeshId > 0) {
                    int baseMeshCount = static_cast<int>(scene.meshes.size());
                    int baseInstanceCount = static_cast<int>(scene.mesh_instances.size());

                    int meshID = getMeshByID(scene, mi.baseMeshId);
                    int instID = getMeshInstanceByID(scene, mi.baseMeshId);
                    if (meshID != -1) {
                        // Base is a mesh
                        mi.baseMeshObjectId = mi.baseMeshId;
                    } 
                    if (instID != -1) {
                        const MeshInstance& baseInst = scene.mesh_instances[instID];
                        mi.baseMeshObjectId = baseInst.baseMeshObjectId;
                    }
                }

                std::cout << mi.baseMeshObjectId << std::endl;


                // --- Store transformations ---
                mi.transformation = instanceTransform;
                mi.inv_transformation = instanceTransform.inverse();

                scene.mesh_instances.push_back(mi);
            };

            if (miJson.is_array()) {
                for (const auto& obj : miJson) parseMeshInstance(obj);
            } else if (miJson.is_object()) {
                parseMeshInstance(miJson);
            }
        }
    }
}

template<typename T>
T swapEndian(T val) {
    uint8_t* bytes = reinterpret_cast<uint8_t*>(&val);
    std::reverse(bytes, bytes + sizeof(T));
    return val;
}

bool Parser::loadPlyMesh(const std::string& filename,
                         std::vector<Vec3r>& vertices,
                         std::vector<Vec3i>& triangles,
                         std::vector<Vec3r>& normals,
                        std::vector<Vec2r>& texCoords)
{
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: could not open PLY file " << filename << std::endl;
        return false;
    }

    std::string line;
    int vertexCount = 0, faceCount = 0;
    enum Format { ASCII, BINARY_LITTLE, BINARY_BIG } format = ASCII;
    
    // --- Track Properties ---
    bool hasNormals = false;
    bool hasUV = false;    
    int uvCount = 0;

    // --- 1. Parse Header Information ---
    while (std::getline(file, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line == "end_header") break;
        
        std::stringstream ss(line);
        std::string keyword;
        ss >> keyword;

        if (keyword == "format") {
            std::string fmtType;
            ss >> fmtType;
            if (fmtType == "binary_little_endian") format = BINARY_LITTLE;
            else if (fmtType == "binary_big_endian") format = BINARY_BIG;
            else format = ASCII;
        }
        else if (keyword == "element") {
            std::string type;
            ss >> type;
            if (type == "vertex") ss >> vertexCount;
            else if (type == "face") ss >> faceCount;
        }
        else if (keyword == "property") {
            std::string type, name;
            ss >> type >> name;
            if (name == "nx") hasNormals = true;

            if (name == "u" || name == "v" || name == "s" || name == "t") {
                hasUV = true;
                uvCount++; 
            }
        }
    }

    if (vertexCount <= 0 || faceCount <= 0) {
        std::cerr << "Error: Invalid PLY header or counts.\n";
        return false;
    }

    vertices.clear(); vertices.reserve(vertexCount);
    triangles.clear(); triangles.reserve(faceCount);
    normals.clear(); if (hasNormals) normals.reserve(vertexCount);
    texCoords.clear(); texCoords.reserve(vertexCount);
    
    vertices.reserve(vertexCount);
    triangles.reserve(faceCount);
    if (hasNormals) normals.reserve(vertexCount); // Reserve memory for normals

    // --- 2. Switch to Binary Mode safely ---
    if (format != ASCII) {
        file.close();
        file.open(filename, std::ios::binary);
        std::string headerLine;
        while (std::getline(file, headerLine)) {
            if (!headerLine.empty() && headerLine.back() == '\r') headerLine.pop_back();
            if (headerLine == "end_header") break;
        }
    }

    std::cout << "Reading " << (format == ASCII ? "ASCII" : "Binary") << " PLY...\n";

    // --- 3. Read Data (UPDATED FOR NORMALS) ---
    for (int i = 0; i < vertexCount; ++i) {
        float x, y, z;
        float nx, ny, nz;
        float skipVal;
        float u = 0, v = 0;

        if (format == ASCII) {
            file >> x >> y >> z;
            if (hasNormals) file >> nx >> ny >> nz;
            if (hasUV) file >> u >> v;
        } else {
            // Read Position
            file.read(reinterpret_cast<char*>(&x), 4);
            file.read(reinterpret_cast<char*>(&y), 4);
            file.read(reinterpret_cast<char*>(&z), 4);
            
            // Read Normals if they exist
            if (hasNormals) {
                file.read(reinterpret_cast<char*>(&nx), 4);
                file.read(reinterpret_cast<char*>(&ny), 4);
                file.read(reinterpret_cast<char*>(&nz), 4);
            }
            if (hasUV) {
                file.read((char*)&u, 4); file.read((char*)&v, 4);
            }

            if (format == BINARY_BIG) {
                x = swapEndian(x); y = swapEndian(y); z = swapEndian(z);
                if (hasNormals) {
                    nx = swapEndian(nx); ny = swapEndian(ny); nz = swapEndian(nz);
                }
            }
        }
        
        vertices.push_back({x, y, z});
        if (hasNormals) {
            normals.push_back({nx, ny, nz});
        }
        if (hasUV) {
            texCoords.push_back({u, v});
        }    
    }

    // --- 4. Read Faces (Standard) ---
    for (int i = 0; i < faceCount; ++i) {
        int nverts;
        std::vector<int> idx;

        if (format == ASCII) {
            file >> nverts;
        } else {
            uint8_t n_bin;
            file.read(reinterpret_cast<char*>(&n_bin), 1);
            nverts = n_bin;
        }

        idx.resize(nverts);
        for (int j = 0; j < nverts; ++j) {
            if (format == ASCII) {
                file >> idx[j];
            } else {
                int32_t val;
                file.read(reinterpret_cast<char*>(&val), 4);
                if (format == BINARY_BIG) val = swapEndian(val);
                idx[j] = val;
            }
        }

        for (int j = 1; j < nverts - 1; ++j) {
            triangles.push_back({idx[0], idx[j], idx[j+1]});
        }
    }

    return true;
}


void Parser::computeVertexNormals(const std::vector<Vec3r>& vertices,
                                  std::vector<Triangle>& triangles)
{
    std::vector<Vec3r> accumulated(vertices.size(), {0, 0, 0});

    // 1. Accumulate face normals per vertex
    for (const auto& tri : triangles) {
        int v0_id = tri.getV0ID();
        int v1_id = tri.getV1ID();
        int v2_id = tri.getV2ID();

        const Vec3r& v0 = vertices[v0_id - 1];
        const Vec3r& v1 = vertices[v1_id - 1];
        const Vec3r& v2 = vertices[v2_id - 1];

        Vec3r faceNormal = crossVec3r(v1 - v0, v2 - v0);

        accumulated[v0_id - 1] = accumulated[v0_id - 1] + faceNormal;
        accumulated[v1_id - 1] = accumulated[v1_id - 1] + faceNormal;
        accumulated[v2_id - 1] = accumulated[v2_id - 1] + faceNormal;
    }

    // 2. Normalize all accumulated normals
    for (auto& n : accumulated)
        n = normalizeVec3r(n);

    // 3. Store per-vertex normals inside each triangle
    for (auto& tri : triangles) {
        tri.setV0Normal(accumulated[tri.getV0ID() - 1]);
        tri.setV1Normal(accumulated[tri.getV1ID() - 1]);
        tri.setV2Normal(accumulated[tri.getV2ID() - 1]);
    }
}



void Parser::checkParsedScene(Scene& scene) {
    std::cout << "--- Scene Parsed Successfully ---" << std::endl;
    std::cout << "Background Color: " << scene.background_color.x << ", " << scene.background_color.y << ", " << scene.background_color.z << std::endl;
    std::cout << "Number of Vertices: " << scene.vertex_data.size() << std::endl;
    std::cout << "Number of Materials: " << scene.materials.size() << std::endl;
    std::cout << "Number of Point Lights: " << scene.point_lights.size() << std::endl;
    std::cout << "Number of Planes: " << scene.planes.size() << std::endl;
    std::cout << "Number of Triangles: " << scene.triangles.size() << std::endl;
    std::cout << "Number of Spheres: " << scene.spheres.size() << std::endl;
    std::cout << "Number of Transformations: " << scene.transformations.size() << std::endl;
}


