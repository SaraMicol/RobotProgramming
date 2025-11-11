#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <yaml-cpp/yaml.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <vector>
#include <string>
#include <memory>
#include <visualization_msgs/MarkerArray.h>

// Include LaserScan, LaserScanner e GridMap
#include "laser_scan.h"
#include "laser_scanner.h"
#include "grid_map.h"
#include "laser_visualization.h"
#include  "robot_config.h"
#include "utils.h"


// --- MAIN ---
int main(int argc, char** argv) {
    ros::init(argc, argv, "robsim_node");
    ros::NodeHandle nh;
    ros::Publisher laser_viz_pub = nh.advertise<visualization_msgs::MarkerArray>("laser_visualization", 1);

    MapConfig map_cfg;
    std::vector<RobotConfig> robots;

    std::string yaml_file = "/home/lattinone/RobotProgramming/src/ros_2d_multi_robot_simulator/configs/env1.yaml";
    parseYAML(yaml_file, map_cfg, robots);

    auto map_msg = createMap(map_cfg);
    auto grid_map = createGridMap(map_msg);

    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
    std::vector<ros::Publisher> pose_pubs;
    std::vector<ros::Publisher> odom_pubs;
    std::vector<ros::Subscriber> cmd_vel_subs;
    
    for (size_t i = 0; i < robots.size(); ++i) {
        pose_pubs.push_back(nh.advertise<geometry_msgs::PoseStamped>("/" + robots[i].id + "/pose", 1));
        odom_pubs.push_back(nh.advertise<nav_msgs::Odometry>("/" + robots[i].id + "/odom", 1));
        
        auto callback = [&robots, i](const geometry_msgs::Twist::ConstPtr& msg) {
            // Aggiorna le velocità in base al cmd_vel ricevuto,
            // la logica di evitamento ostacoli le sovrascriverà se necessario.
            robots[i].current_v_lin = msg->linear.x;
            robots[i].current_v_ang = msg->angular.z;
        };
        
        cmd_vel_subs.push_back(nh.subscribe<geometry_msgs::Twist>(
            "/" + robots[i].id + "/cmd_vel", 10, callback));
        
        ROS_INFO("Robot %s: Subscribed to /%s/cmd_vel", robots[i].id.c_str(), robots[i].id.c_str());
    }

    std::vector<ActiveSensor> active_sensors;
    for (size_t ri = 0; ri < robots.size(); ++ri) {
        for (const auto &s : robots[ri].sensors) {
            if (s.type == "lidar" || s.type == "laser") {
                ActiveSensor as;
                as.cfg = s;
                as.robot_index = ri;
                as.pub = nh.advertise<sensor_msgs::LaserScan>(s.topic, 1);
                
                as.laser_scan = std::make_shared<LaserScan>(
                    s.range_min, s.range_max, 
                    s.angle_min, s.angle_max, 
                    s.beams
                );
                
                active_sensors.push_back(as);
            }
        }
    }

    tf2_ros::TransformBroadcaster tf_broadcaster;
    ros::Rate rate(10);
    float dt = 0.1;
    
    ROS_INFO("Simulation started. Waiting for cmd_vel commands...");
    
    static int counter = 0;
    while (ros::ok()) {
    ros::Time now = ros::Time::now();
    
    // Simula LaserScan
    for (auto &as : active_sensors) {
        const auto &rob = robots[as.robot_index];
        
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
        
        for (size_t i = 0; i < as.laser_scan->ranges.size(); ++i) {
            float beam_angle = as.laser_scan->angle_min + angle_increment * i;
            Vector2f d(cos(beam_angle), sin(beam_angle));
            d = sensor_pose.linear() * d;
            
            as.laser_scan->ranges[i] = grid_map->scanRay(
                sensor_pose.translation(), 
                d, 
                as.laser_scan->range_max
            );
        }
        
        sensor_msgs::LaserScan scan_msg;
        scan_msg.header.stamp = now;
        scan_msg.header.frame_id = as.cfg.frame_id;
        scan_msg.angle_min = as.laser_scan->angle_min;
        scan_msg.angle_max = as.laser_scan->angle_max;
        scan_msg.angle_increment = angle_increment;
        scan_msg.time_increment = 0.0;
        scan_msg.scan_time = dt;
        scan_msg.range_min = as.laser_scan->range_min;
        scan_msg.range_max = as.laser_scan->range_max;
        scan_msg.ranges = as.laser_scan->ranges;
        
        as.pub.publish(scan_msg);
    }
    publishLaserVisualization(laser_viz_pub, active_sensors, robots, now);

    // Aggiorna cinematica di tutti i robot (SENZA evitamento ostacoli)
    for (size_t i = 0; i < robots.size(); ++i) { 
        updateUnicycleKinematics(robots[i], dt, grid_map);
        
        if (counter % 10 == 0) {
            ROS_INFO("Robot %s: pos=(%.2f, %.2f) theta=%.2f vel=(%.2f, %.2f)", 
                     robots[i].id.c_str(), robots[i].x, robots[i].y, robots[i].alpha, 
                     robots[i].current_v_lin, robots[i].current_v_ang);
        }
    }
    counter++;
    
    map_msg.header.stamp = now;
    map_pub.publish(map_msg);
    publishTF(tf_broadcaster, robots);

    for (size_t i = 0; i < robots.size(); ++i) {
        geometry_msgs::PoseStamped p;
        p.header.stamp = now;
        p.header.frame_id = "map";
        p.pose.position.x = robots[i].x;
        p.pose.position.y = robots[i].y;
        tf2::Quaternion q;
        q.setRPY(0, 0, robots[i].alpha);
        p.pose.orientation.x = q.x();
        p.pose.orientation.y = q.y();
        p.pose.orientation.z = q.z();
        p.pose.orientation.w = q.w();
        pose_pubs[i].publish(p);
        
        nav_msgs::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = "odom";
        odom.child_frame_id = robots[i].frame_id;
        odom.pose.pose = p.pose;
        odom.twist.twist.linear.x = robots[i].current_v_lin;
        odom.twist.twist.angular.z = robots[i].current_v_ang;
        odom_pubs[i].publish(odom);
    }

    ros::spinOnce();
    rate.sleep(); 
  }

    return 0;
}