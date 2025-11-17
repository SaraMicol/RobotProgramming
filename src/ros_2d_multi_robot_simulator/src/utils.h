#pragma once

#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <string>
#include <memory>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "robot_config.h"
#include "grid_map.h"
#include <yaml-cpp/yaml.h>

struct MapConfig {
    int width, height;
    float resolution;
    float origin_x, origin_y;
    std::vector<std::vector<float>> obstacles;
    std::string image_file; 
}; 

// Dichiarazioni delle funzioni
void parseYAML(const std::string &file, MapConfig &map_cfg, std::vector<RobotConfig> &robots);
nav_msgs::OccupancyGrid createMap(const MapConfig &map_cfg);
std::shared_ptr<GridMap> createGridMap(const nav_msgs::OccupancyGrid& map_msg);
void updateUnicycleKinematics(RobotConfig &robot, float dt, const std::shared_ptr<GridMap>& grid_map);
void publishTF(tf2_ros::TransformBroadcaster &tf_broadcaster, const std::vector<RobotConfig> &robots);
double computeObstacleAvoidance(const RobotConfig& robot, const ActiveSensor& sensor);

