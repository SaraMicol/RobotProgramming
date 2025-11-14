#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <string>
#include <vector>
#include <memory>
#include <ros/ros.h>
#include "laser_scan.h"

struct SensorConfig {
    std::string type, frame_id, topic;
    int beams;
    float range_min, range_max;
    float angle_min, angle_max;
};

struct RobotConfig {
    std::string id, frame_id;
    float x, y, alpha, radius, v_lin, v_ang;
    std::vector<SensorConfig> sensors;
    float current_v_lin = 0.0;
    float current_v_ang = 0.0;
    int index;
};

struct ActiveSensor {
    SensorConfig cfg;
    ros::Publisher pub;
    size_t robot_index;
    std::shared_ptr<LaserScan> laser_scan;
};

#endif