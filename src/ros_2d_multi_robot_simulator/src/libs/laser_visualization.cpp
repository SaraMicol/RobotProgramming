#include "laser_visualization.h"
#include "robot_config.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

using Vector2f = Eigen::Vector2f;
using Isometry2f = Eigen::Isometry2f;



void publishLaserVisualization(ros::Publisher& marker_pub,
                               const std::vector<ActiveSensor>& active_sensors,
                               const std::vector<RobotConfig>& robots,
                               const ros::Time& now) {
    visualization_msgs::MarkerArray marker_array;
    int marker_id = 0;
    
    for (const auto& as : active_sensors) {
        const auto& rob = robots[as.robot_index];
        
        // Calcola la posizione del sensore
        Isometry2f robot_pose = Isometry2f::Identity();
        robot_pose.translation() = Vector2f(rob.x, rob.y);
        Eigen::Rotation2Df rot(rob.alpha);
        robot_pose.linear() = rot.toRotationMatrix();
        
        Vector2f sensor_offset(rob.radius, 0);
        Vector2f sensor_global_pos = robot_pose * sensor_offset;
        
        Isometry2f sensor_pose = robot_pose;
        sensor_pose.translation() = sensor_global_pos;
        
        float angle_increment = (as.laser_scan->angle_max - as.laser_scan->angle_min) / 
                               as.laser_scan->ranges.size();
        
        // Crea un marker LINE_LIST per tutti i raggi
        visualization_msgs::Marker laser_rays;
        laser_rays.header.frame_id = "map";
        laser_rays.header.stamp = now;
        laser_rays.ns = "laser_rays_" + rob.id;
        laser_rays.id = marker_id++;
        laser_rays.type = visualization_msgs::Marker::LINE_LIST;
        laser_rays.action = visualization_msgs::Marker::ADD;
        laser_rays.scale.x = 0.01; // Spessore linea
        laser_rays.color.r = 1.0;
        laser_rays.color.g = 0.0;
        laser_rays.color.b = 0.0;
        laser_rays.color.a = 0.5;
        laser_rays.lifetime = ros::Duration(0.15);
        
        // Aggiungi ogni raggio
        for (size_t i = 0; i < as.laser_scan->ranges.size(); ++i) {
            float beam_angle = as.laser_scan->angle_min + angle_increment * i;
            Vector2f d(cos(beam_angle), sin(beam_angle));
            d = sensor_pose.linear() * d;
            
            float range = as.laser_scan->ranges[i];
            
            // Punto iniziale (posizione sensore)
            geometry_msgs::Point p_start;
            p_start.x = sensor_global_pos.x();
            p_start.y = sensor_global_pos.y();
            p_start.z = 0.1;
            
            // Punto finale (dove il raggio colpisce)
            geometry_msgs::Point p_end;
            p_end.x = sensor_global_pos.x() + d.x() * range;
            p_end.y = sensor_global_pos.y() + d.y() * range;
            p_end.z = 0.1;
            
            laser_rays.points.push_back(p_start);
            laser_rays.points.push_back(p_end);
            
            // Colora diversamente i raggi
            std_msgs::ColorRGBA color;
            if (range < rob.radius + 0.5) {
                color.r = 1.0; color.g = 0.0; color.b = 0.0; color.a = 0.8;
            } else if (range < rob.radius + 1.0) {
                color.r = 1.0; color.g = 1.0; color.b = 0.0; color.a = 0.6;
            } else {
                color.r = 0.0; color.g = 1.0; color.b = 0.0; color.a = 0.4;
            }
            laser_rays.colors.push_back(color);
            laser_rays.colors.push_back(color);
        }
        
        marker_array.markers.push_back(laser_rays);
        
        // Marker SPHERE alla posizione del sensore
        visualization_msgs::Marker sensor_marker;
        sensor_marker.header.frame_id = "map";
        sensor_marker.header.stamp = now;
        sensor_marker.ns = "laser_sensor_" + rob.id;
        sensor_marker.id = marker_id++;
        sensor_marker.type = visualization_msgs::Marker::SPHERE;
        sensor_marker.action = visualization_msgs::Marker::ADD;
        sensor_marker.pose.position.x = sensor_global_pos.x();
        sensor_marker.pose.position.y = sensor_global_pos.y();
        sensor_marker.pose.position.z = 0.1;
        sensor_marker.scale.x = 0.05;
        sensor_marker.scale.y = 0.05;
        sensor_marker.scale.z = 0.05;
        sensor_marker.color.r = 0.0;
        sensor_marker.color.g = 0.0;
        sensor_marker.color.b = 1.0;
        sensor_marker.color.a = 1.0;
        sensor_marker.lifetime = ros::Duration(0.15);
        
        marker_array.markers.push_back(sensor_marker);
    }
    
    marker_pub.publish(marker_array);
}