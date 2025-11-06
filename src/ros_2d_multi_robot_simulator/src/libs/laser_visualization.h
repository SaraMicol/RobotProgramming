#ifndef LASER_VISUALIZATION_H
#define LASER_VISUALIZATION_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <vector>

// Forward declarations (dichiarazioni anticipate)
struct SensorConfig;
struct RobotConfig;
struct ActiveSensor;

// Dichiarazione della funzione
void publishLaserVisualization(ros::Publisher& marker_pub,
                               const std::vector<ActiveSensor>& active_sensors,
                               const std::vector<RobotConfig>& robots,
                               const ros::Time& now);

#endif // LASER_VISUALIZATION_H