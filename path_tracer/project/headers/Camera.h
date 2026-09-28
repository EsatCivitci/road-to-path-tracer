#pragma once

#include "Real.h"
#include "ToneMap.h"
#include <iostream>

class Camera {
public:
    Camera() = default;
    ~Camera() = default;

    // Getters!!
    int getID() const { return ID; }
    const Vec3r& getPosition() const { return position; }
    const Vec3r& getGaze() const { return gaze; }
    const Vec3r& getUp() const { return up; }
    const std::vector<real>& getNearPlane() const { return near_plane; }
    real getNearDistance() const { return near_distance; }
    const Vec2i& getImageResolution() const { return image_resolution; }
    const std::string& getImageName() const { return image_name; }
    const real getApertureSize() const { return aperture_size; }
    const int getNumSamples() const { return num_samples; }
    const real getFocusDistance() const { return focus_distance; }
    const ToneMap getTM() const { return tm; }
    const std::string getRenderer() const {return renderer;}

    // Setters!!
    void setID(int i_ID) { ID = i_ID; }
    void setPosition(const Vec3r& i_position) { position = i_position; }
    void setGaze(const Vec3r& i_gaze) { gaze = i_gaze; }
    void setUp(const Vec3r& i_up) { up = i_up; }
    void setNearPlane(const std::vector<real>& i_near_plane) { near_plane = i_near_plane; }
    void setNearDistance(real i_near_distance) { near_distance = i_near_distance; }
    void setImageResolution(const Vec2i& i_image_resolution) { image_resolution = i_image_resolution; }
    void setImageName(const std::string& i_image_name) { image_name = i_image_name; }
    void setApertureSize(const real i_aperture_size) { aperture_size = i_aperture_size; }
    void setNumSamples(const int i_num_samples) { num_samples = i_num_samples; }
    void setFocusDistance(const real i_focus_distance) { focus_distance = i_focus_distance; }
    void setTM (const ToneMap& i_tm) { tm = i_tm; }
    void setRenderer(const std::string i_renderer) {renderer = i_renderer;}

    std::vector<std::string>& getTransformIds() { return transformIds; }
    bool isTransformed = false;
private:
    int ID = -1;
    Vec3r position = {0,0,0};
    Vec3r gaze = {0,0,0};
    Vec3r up = {0,0,0};
    std::vector<real> near_plane;
    real near_distance = -1;
    Vec2i image_resolution;
    std::string image_name;
    std::vector<std::string> transformIds;
    real aperture_size = -1;
    int num_samples = -1;
    real focus_distance = -1;
    ToneMap tm;
    std::string renderer = "ApplyShading";
};