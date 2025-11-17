#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <yaml-cpp/yaml.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <vector>
#include <string>
#include <memory>
#include <Eigen/Geometry>
#include <boost/bind.hpp>

#include "laser_scan.h"    
#include "laser_scanner.h" 
#include "grid_map.h"      
#include "robot_config.h"  
#include "utils.h"         

struct RobotGoal {
    bool has_goal;
    double goal_x;
    double goal_y;
    
    RobotGoal() : has_goal(false), goal_x(0.0), goal_y(0.0) {}
};

std::vector<RobotGoal> robot_goals;
std::vector<ActiveSensor> active_sensors;

// Dichiarazioni forward (assicurati che esistano in utils.h o in un file appropriato)
double computeObstacleAvoidance(const RobotConfig& robot, const ActiveSensor& sensor);
void updateUnicycleKinematics(RobotConfig& robot, float dt, const std::unique_ptr<GridMap>& map);
void parseYAML(const std::string& filename, MapConfig& map_cfg, std::vector<RobotConfig>& robots);
nav_msgs::OccupancyGrid createMap(const MapConfig& cfg); 
nav_msgs::OccupancyGrid toOccupancyGrid(const GridMap& grid_map); // NECESSARIA PER LA CONVERSIONE

void updateCmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg, RobotConfig* robot)
{
    robot->current_v_lin = msg->linear.x;
    robot->current_v_ang = msg->angular.z;
}

void updateRobotControl(RobotConfig* robot, RobotGoal* robot_goal)
{
    if (!robot_goal->has_goal) {
        robot->current_v_lin = 0.0;
        robot->current_v_ang = 0.0;
        return;
    }
    
    double dx = robot_goal->goal_x - robot->x;
    double dy = robot_goal->goal_y - robot->y;
    double dist = std::sqrt(dx*dx + dy*dy);

    const double GOAL_TOLERANCE = 0.15;
    if (dist < GOAL_TOLERANCE) {
        robot->current_v_lin = 0.0;
        robot->current_v_ang = 0.0;
        robot_goal->has_goal = false;
        ROS_INFO("Robot %s: Goal raggiunto!", robot->id.c_str());
        return;
    }

    double target_theta = std::atan2(dy, dx);
    double angle_diff = target_theta - robot->alpha;
    while(angle_diff > M_PI) angle_diff -= 2.0*M_PI;
    while(angle_diff < -M_PI) angle_diff += 2.0*M_PI;

    const double MAX_LINEAR_VEL = 0.5;
    const double MAX_ANGULAR_VEL = 1.0;
    const double ANGULAR_THRESHOLD = 0.3;

    if (std::fabs(angle_diff) > ANGULAR_THRESHOLD) {
        robot->current_v_lin = 0.0;
        robot->current_v_ang = std::min(MAX_ANGULAR_VEL, 
                                       std::max(-MAX_ANGULAR_VEL, 3.0 * angle_diff));
    } else {
        double avoid = 0.0;
        for (const auto& s : active_sensors) {
            if (s.robot_index == robot->index) {
                avoid = computeObstacleAvoidance(*robot, s);
                break;
            }
        }
        double angular = 2.0 * angle_diff + avoid;
        robot->current_v_lin = std::min(MAX_LINEAR_VEL, 0.8 * dist);
        if (std::fabs(avoid) > 0.2) robot->current_v_lin *= 0.4;
        robot->current_v_ang = std::min(MAX_ANGULAR_VEL,
                                       std::max(-MAX_ANGULAR_VEL, angular));
    }
}

double computeObstacleAvoidance(const RobotConfig& robot, const ActiveSensor& sensor)
{
    double avoid_turn = 0.0;
    const double OBSTACLE_THRESHOLD = 0.6;
    const double TURN_GAIN = 1.2;

    const auto& ranges = sensor.scanner->scan.ranges;
    int N = ranges.size();
    if (N == 0) return 0.0;

    double angle_increment = (sensor.scanner->scan.angle_max - sensor.scanner->scan.angle_min) / (N - 1);

    for (int i = 0; i < N; i++) {
        float d = ranges[i];
        if (d < OBSTACLE_THRESHOLD && d > 0.01) {
            float angle = sensor.scanner->scan.angle_min + i * angle_increment;
            avoid_turn -= TURN_GAIN * (OBSTACLE_THRESHOLD - d) * std::sin(angle);
        }
    }
    return avoid_turn;
}

void publishLaserTransform(tf2_ros::TransformBroadcaster& tf_broadcaster,
                          const std::string& robot_frame_id, 
                          const std::string& laser_frame_id, 
                          const Eigen::Isometry2f& laser_pose) {
    ros::Time current_time = ros::Time::now();

    geometry_msgs::TransformStamped laser_transform;
    laser_transform.header.stamp = current_time;
    laser_transform.header.frame_id = robot_frame_id;
    laser_transform.child_frame_id = laser_frame_id;
    laser_transform.transform.translation.x = laser_pose.translation().x();
    laser_transform.transform.translation.y = laser_pose.translation().y();
    laser_transform.transform.translation.z = 0.0;

    float yaw = std::atan2(laser_pose.rotation()(1, 0), laser_pose.rotation()(0, 0));
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    laser_transform.transform.rotation.x = q.x();
    laser_transform.transform.rotation.y = q.y();
    laser_transform.transform.rotation.z = q.z();
    laser_transform.transform.rotation.w = q.w();

    tf_broadcaster.sendTransform(laser_transform);
}

void publishRobotTransform(tf2_ros::TransformBroadcaster& tf_broadcaster, const std::vector<RobotConfig>& robots) {
    ros::Time current_time = ros::Time::now();

    for (const auto& robot : robots) {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = current_time;
        transformStamped.header.frame_id = "map"; 
        transformStamped.child_frame_id = robot.id + "_frame"; 

        transformStamped.transform.translation.x = robot.x;
        transformStamped.transform.translation.y = robot.y;
        transformStamped.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, robot.alpha);
        transformStamped.transform.rotation = tf2::toMsg(q);

        tf_broadcaster.sendTransform(transformStamped);
    }
}

// IMPLEMENTAZIONE DI toOccupancyGrid SPOSTATA QUI
nav_msgs::OccupancyGrid toOccupancyGrid(const GridMap& grid_map) {
    nav_msgs::OccupancyGrid map_msg;
    map_msg.header.frame_id = "map";
    
    map_msg.info.width = grid_map.cols;
    map_msg.info.height = grid_map.rows;
    map_msg.info.resolution = grid_map.resolution();
    
    // Assumendo che GridMap::origin() restituisca Vector2f
    using Vector2f = Eigen::Vector2f; 
    Vector2f origin = grid_map.origin(); 
    map_msg.info.origin.position.x = origin.x();
    map_msg.info.origin.position.y = origin.y();
    map_msg.info.origin.position.z = 0.0;
    map_msg.info.origin.orientation.w = 1.0;

    map_msg.data.resize(grid_map.rows * grid_map.cols);

    for (int r = 0; r < grid_map.rows; ++r) {
        for (int c = 0; c < grid_map.cols; ++c) {
            int idx = r * grid_map.cols + c;
            int8_t map_value = grid_map.cells[idx]; 
            
            map_msg.data[idx] = (map_value == 100) ? 100 : 0; 
        }
    }
    return map_msg;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "robsim_node");
    ros::NodeHandle nh;
    
    MapConfig map_cfg;
    std::vector<RobotConfig> robots;
    std::string yaml_file = "/home/lattinone/RobotProgramming/src/ros_2d_multi_robot_simulator/configs/env1.yaml";
    
    parseYAML(yaml_file, map_cfg, robots); 
    
    // --- INIZIALIZZAZIONE MAPPA DA IMMAGINE ---
    const std::string image_path = "/home/lattinone/RobotProgramming/src/ros_2d_multi_robot_simulator/maps/cappero_laser_odom_diag_2020-05-06-16-26-03.png";

    auto grid_map = std::make_unique<GridMap>(map_cfg.resolution); 
    
    try {
        grid_map->loadFromImage(image_path.c_str(), map_cfg.resolution);
        ROS_INFO("Mappa caricata da immagine: %s", image_path.c_str());
    } catch (const std::exception& e) {
        ROS_ERROR("Errore nel caricamento dell'immagine: %s", e.what());
        return 1;
    }
    
    // Converte GridMap in OccupancyGrid
    nav_msgs::OccupancyGrid map_msg = toOccupancyGrid(*grid_map);
    // --- FINE INIZIALIZZAZIONE MAPPA ---

    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
    map_msg.header.stamp = ros::Time::now();
    map_msg.header.frame_id = "map";
    map_pub.publish(map_msg);
    ROS_INFO("Mappa pubblicata su /map (%dx%d, res=%.3f)", 
             map_msg.info.width, map_msg.info.height, map_msg.info.resolution);

    ros::Duration(0.5).sleep();
    ros::spinOnce();

    robot_goals.resize(robots.size());

    std::vector<Eigen::Isometry2f> robot_poses(robots.size());
    
    for (size_t i = 0; i < robots.size(); ++i) {
        robot_poses[i] = Eigen::Isometry2f::Identity();
        robot_poses[i].translation() = Eigen::Vector2f(robots[i].x, robots[i].y);
        robot_poses[i].linear() = Eigen::Rotation2Df(robots[i].alpha).toRotationMatrix();
    }

    std::vector<ros::Publisher> pose_pubs;
    std::vector<ros::Publisher> odom_pubs;
    std::vector<ros::Subscriber> cmd_vel_subs;
    
    for (size_t i = 0; i < robots.size(); ++i) {
        pose_pubs.push_back(nh.advertise<geometry_msgs::PoseStamped>("/" + robots[i].id + "/pose", 1));
        odom_pubs.push_back(nh.advertise<nav_msgs::Odometry>("/" + robots[i].id + "/odom", 50));
        cmd_vel_subs.push_back(nh.subscribe<geometry_msgs::Twist>(
            "/" + robots[i].id + "/cmd_vel", 10,
            boost::bind(&updateCmdVelCallback, _1, &robots[i])
        ));
        ROS_INFO("Robot %s: Pronto.", robots[i].id.c_str());
    }
    
    ros::Subscriber global_goal_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/move_base_simple/goal", 10,
        [&](const geometry_msgs::PoseStamped::ConstPtr& goal_msg){
            ROS_INFO("Goal globale ricevuto (%.2f, %.2f)",
                     goal_msg->pose.position.x, goal_msg->pose.position.y);
            for (size_t i = 0; i < robots.size(); ++i) {
                robot_goals[i].has_goal = true;
                robot_goals[i].goal_x = goal_msg->pose.position.x;
                robot_goals[i].goal_y = goal_msg->pose.position.y;
            }
        }
    );
    
    for (size_t ri = 0; ri < robots.size(); ++ri) {
        for (const auto &s : robots[ri].sensors) {
            if (s.type == "lidar" || s.type == "laser") {
                ActiveSensor as;
                as.cfg = s;
                as.robot_index = ri;
                as.pub = nh.advertise<sensor_msgs::LaserScan>(s.topic, 50);
                as.frame_id = s.frame_id;
                
                auto scan_config = std::make_shared<LaserScan>(
                    s.range_min, s.range_max, 
                    s.angle_min, s.angle_max, 
                    s.beams 
                );
                as.laser_scan = scan_config; 
                
                Eigen::Isometry2f scanner_in_robot = Eigen::Isometry2f::Identity();
                scanner_in_robot.translation().x() = robots[ri].radius;
                
                as.scanner = std::make_shared<LaserScanner>(
                    *scan_config,
                    &robot_poses[ri],     
                    grid_map.get(),       
                    scanner_in_robot,     
                    10.0f                 
                );
                as.scanner->radius = 0.1;
                
                active_sensors.push_back(as);
                ROS_INFO("Robot %s: Laser su %s (Frame: %s)", 
                         robots[ri].id.c_str(), s.topic.c_str(), s.frame_id.c_str());
            }
        }
    }

    tf2_ros::TransformBroadcaster tf_broadcaster;
    
    ros::Rate rate(10);
    float dt = 0.1;
    ROS_INFO("Simulazione avviata. Invia goal su /move_base_simple/goal.");

    while (ros::ok()) {
        ros::Time now = ros::Time::now();

        for (size_t i = 0; i < robots.size(); ++i) {
            robot_poses[i].translation() = Eigen::Vector2f(robots[i].x, robots[i].y);
            robot_poses[i].linear() = Eigen::Rotation2Df(robots[i].alpha).toRotationMatrix();
        }

        publishRobotTransform(tf_broadcaster, robots); 

        for (auto &as : active_sensors) {
            as.scanner->tick(dt);
            
            if (as.scanner->scan_ready) {
                sensor_msgs::LaserScan scan_msg;
                scan_msg.header.stamp = now;
                scan_msg.header.frame_id = as.frame_id;
                scan_msg.angle_min = as.scanner->scan.angle_min;
                scan_msg.angle_max = as.scanner->scan.angle_max;
                scan_msg.angle_increment = 
                    (as.scanner->scan.angle_max - as.scanner->scan.angle_min) / 
                    (as.scanner->scan.ranges.size() - 1);
                scan_msg.range_min = as.scanner->scan.range_min;
                scan_msg.range_max = as.scanner->scan.range_max;
                scan_msg.ranges = as.scanner->scan.ranges;
                scan_msg.intensities.clear();
                
                as.pub.publish(scan_msg);
                
                Eigen::Isometry2f lidar_offset = Eigen::Isometry2f::Identity();
                lidar_offset.translation() = as.scanner->pose_in_parent.translation();
                lidar_offset.linear() = as.scanner->pose_in_parent.linear();

                publishLaserTransform(
                    tf_broadcaster,
                    robots[as.robot_index].id + "_frame", 
                    as.frame_id,                          
                    lidar_offset                           
                );
            }
        }

        for (size_t i = 0; i < robots.size(); ++i) {
            updateRobotControl(&robots[i], &robot_goals[i]);
            // updateUnicycleKinematics deve essere definito in utils.cpp!
            updateUnicycleKinematics(robots[i], dt, grid_map);
        }

        map_msg.header.stamp = now;
        map_pub.publish(map_msg);

        for (size_t i = 0; i < robots.size(); ++i) {
            geometry_msgs::PoseStamped p;
            p.header.stamp = now;
            p.header.frame_id = "map"; 
            p.pose.position.x = robots[i].x;
            p.pose.position.y = robots[i].y;

            tf2::Quaternion q;
            q.setRPY(0, 0, robots[i].alpha);
            p.pose.orientation = tf2::toMsg(q);

            pose_pubs[i].publish(p);

            nav_msgs::Odometry odom;
            odom.header.stamp = now;
            odom.header.frame_id = "odom"; 
            odom.child_frame_id = robots[i].id + "_frame"; 
            odom.pose.pose.position = p.pose.position; 
            odom.pose.pose.orientation = p.pose.orientation;
            odom.twist.twist.linear.x = robots[i].current_v_lin;
            odom.twist.twist.angular.z = robots[i].current_v_ang;

            odom_pubs[i].publish(odom);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}