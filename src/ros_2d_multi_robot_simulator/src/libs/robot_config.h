#pragma once

#include <string>
#include <vector>
#include <memory>
#include <ros/ros.h>

// ✅ AGGIUNGI FORWARD DECLARATION
class LaserScanner;
class LaserScan;

// Configurazione sensore
struct SensorConfig {
    std::string type;
    std::string frame_id;
    std::string topic;
    int beams;
    float range_min;
    float range_max;
    float angle_min;
    float angle_max;
    
    SensorConfig() : beams(0), range_min(0.0f), range_max(0.0f), 
                     angle_min(0.0f), angle_max(0.0f) {}
};

// Configurazione robot
struct RobotConfig {
    std::string id;
    std::string frame_id;
    int index;
    float x, y, alpha;
    float radius;
    float v_lin, v_ang;
    float current_v_lin, current_v_ang;
    std::vector<SensorConfig> sensors;
    
    RobotConfig() : index(0), x(0), y(0), alpha(0), radius(0.2f),
                    v_lin(0), v_ang(0), current_v_lin(0), current_v_ang(0) {}
};

// Sensore attivo
struct ActiveSensor {
    SensorConfig cfg;
    int robot_index;
    ros::Publisher pub;
    std::string frame_id;
    
    // ✅ Ora il compilatore sa che LaserScanner esiste
    std::shared_ptr<LaserScanner> scanner;
    std::shared_ptr<LaserScan> laser_scan;
    
    ActiveSensor() : robot_index(-1) {}
};