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

#include "laser_scan.h"
#include "laser_scanner.h"
#include "grid_map.h"
#include "robot_config.h"
#include "utils.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


// Struttura per memorizzare stato goal per ogni robot
struct RobotGoal {
    bool has_goal;
    double goal_x;
    double goal_y;
    
    RobotGoal() : has_goal(false), goal_x(0.0), goal_y(0.0) {}
};

std::vector<RobotGoal> robot_goals;
std::vector<ActiveSensor> active_sensors;

// Dichiarazione forward della funzione
double computeObstacleAvoidance(const RobotConfig& robot, const ActiveSensor& sensor);

// Callback per cmd_vel (override manuale)
void updateCmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg, RobotConfig* robot)
{
    robot->current_v_lin = msg->linear.x;
    robot->current_v_ang = msg->angular.z;
}

// Aggiorna controllo robot verso goal
void updateRobotControl(RobotConfig* robot, RobotGoal* robot_goal)
{
    // se il robot non ha un goal lo fermo
    if (!robot_goal->has_goal) {
        robot->current_v_lin = 0.0;
        robot->current_v_ang = 0.0;
        return;
    }
    
    //Calcola la distanza tra la posizione attuale del robot (x,y) e il goal (goal_x, goal_y).
    double dx = robot_goal->goal_x - robot->x;
    double dy = robot_goal->goal_y - robot->y;
    double dist = sqrt(dx*dx + dy*dy);

    //Se il robot è vicino abbastanza al goal (0.15 m), ferma il robot e segna il goal come raggiunto.
    const double GOAL_TOLERANCE = 0.15;
    if (dist < GOAL_TOLERANCE) {
        robot->current_v_lin = 0.0;
        robot->current_v_ang = 0.0;
        robot_goal->has_goal = false;
        ROS_INFO("Robot %s: Goal raggiunto!", robot->id.c_str());
        return;
    }

    // angolo verso il goal e quanto robot deve ruotare per raggiungere goal
    double target_theta = atan2(dy, dx);
    double angle_diff = target_theta - robot->alpha;
    while(angle_diff > M_PI) angle_diff -= 2.0*M_PI;
    while(angle_diff < -M_PI) angle_diff += 2.0*M_PI;

    const double MAX_LINEAR_VEL = 0.5;
    const double MAX_ANGULAR_VEL = 1.0;
    const double ANGULAR_THRESHOLD = 0.3;

    if (fabs(angle_diff) > ANGULAR_THRESHOLD) {
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
        if (fabs(avoid) > 0.2) robot->current_v_lin *= 0.4;
        robot->current_v_ang = std::min(MAX_ANGULAR_VEL,
                                       std::max(-MAX_ANGULAR_VEL, angular));
    }
}

// Evitamento ostacoli
double computeObstacleAvoidance(const RobotConfig& robot, const ActiveSensor& sensor)
{
    double avoid_turn = 0.0;
    const double OBSTACLE_THRESHOLD = 0.6;
    const double TURN_GAIN = 1.2;

    const auto& ranges = sensor.laser_scan->ranges;
    int N = ranges.size();
    if (N == 0) return 0.0;

    double angle_increment = (sensor.laser_scan->angle_max - sensor.laser_scan->angle_min) / N;

    for (int i = 0; i < N; i++) {
        float d = ranges[i];
        if (d < OBSTACLE_THRESHOLD && d > 0.01) {
            float angle = sensor.laser_scan->angle_min + i * angle_increment;
            avoid_turn -= TURN_GAIN * (OBSTACLE_THRESHOLD - d) * sin(angle);
        }
    }
    return avoid_turn;
}

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "robsim_node");
    ros::NodeHandle nh;

    ROS_INFO("Inizializzazione nodo robsim_node...");

    // --- Percorso YAML e lettura robot ---
    std::string yaml_file = "/home/lattinone/RobotProgramming/src/ros_2d_multi_robot_simulator/configs/env1.yaml";
    MapConfig map_cfg;  
    std::vector<RobotConfig> robots;
    parseYAML(yaml_file, map_cfg, robots);
    ROS_INFO("YAML letto: %lu robot caricati, risoluzione mappa %.3f", robots.size(), map_cfg.resolution);

    // --- Percorso mappa immagine ---
    std::string map_image_path = "/home/lattinone/RobotProgramming/src/ros_2d_multi_robot_simulator/maps/cappero_laser_odom_diag_2020-05-06-16-26-03.png";
    ROS_INFO("Mappa caricata dal file: %s", map_image_path.c_str());

    // --- Inizializza GridMap ---
    std::shared_ptr<GridMap> grid_map = std::make_shared<GridMap>(0, 0, map_cfg.resolution);
    grid_map->loadFromImage(map_image_path.c_str(), map_cfg.resolution);
    ROS_INFO("GridMap inizializzata: %d x %d celle, risoluzione %.3f", grid_map->rows, grid_map->cols, grid_map->resolution());

    // --- Pubblica OccupancyGrid ---
    nav_msgs::OccupancyGrid map_msg;
    map_msg.header.frame_id = "map";
    map_msg.header.stamp = ros::Time::now();
    map_msg.info.resolution = grid_map->resolution();
    map_msg.info.width = grid_map->cols;
    map_msg.info.height = grid_map->rows;
    map_msg.data.resize(grid_map->rows * grid_map->cols);
    for (int row = 0; row < grid_map->rows; ++row) {
        for (int col = 0; col < grid_map->cols; ++col) {
            uint8_t value = (*grid_map)(row, col);
            map_msg.data[row * grid_map->cols + col] = (value < 127 ? 100 : 0);
        }
    }
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
    map_pub.publish(map_msg);
    ROS_INFO("OccupancyGrid pubblicata su /map (%dx%d)", map_msg.info.width, map_msg.info.height);

    ros::Duration(0.5).sleep();
    ros::spinOnce();

    // --- Inizializza stato goal robot ---
    robot_goals.resize(robots.size());

    // --- Creazione publisher/subscriber per robot ---
    std::vector<ros::Publisher> pose_pubs, odom_pubs;
    std::vector<ros::Subscriber> cmd_vel_subs;
    for (size_t i = 0; i < robots.size(); ++i) {
        pose_pubs.push_back(nh.advertise<geometry_msgs::PoseStamped>("/" + robots[i].id + "/pose", 1));
        odom_pubs.push_back(nh.advertise<nav_msgs::Odometry>("/" + robots[i].id + "/odom", 50));
        cmd_vel_subs.push_back(nh.subscribe<geometry_msgs::Twist>(
            "/" + robots[i].id + "/cmd_vel", 10,
            boost::bind(&updateCmdVelCallback, _1, &robots[i])
        ));
        ROS_INFO("Robot %s pronto.", robots[i].id.c_str());
    }

    // --- Subscriber per goal globale ---
    ros::Subscriber global_goal_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/move_base_simple/goal", 10,
        [&](const geometry_msgs::PoseStamped::ConstPtr& goal_msg){
            ROS_INFO("Goal globale ricevuto: (%.2f, %.2f)", goal_msg->pose.position.x, goal_msg->pose.position.y);
            for (size_t i = 0; i < robots.size(); ++i) {
                robot_goals[i].has_goal = true;
                robot_goals[i].goal_x = goal_msg->pose.position.x;
                robot_goals[i].goal_y = goal_msg->pose.position.y;
            }
        }
    );

    // --- Inizializzazione laser ---
    for (size_t ri = 0; ri < robots.size(); ++ri) {
        for (const auto &s : robots[ri].sensors) {
            if (s.type == "lidar" || s.type == "laser") {
                ActiveSensor as;
                as.cfg = s;
                as.robot_index = ri;
                as.pub = nh.advertise<sensor_msgs::LaserScan>(s.topic, 50);
                as.frame_id = s.frame_id;
                as.laser_scan = std::make_shared<LaserScan>(s.range_min, s.range_max, s.angle_min, s.angle_max, s.beams);
                active_sensors.push_back(as);
                ROS_INFO("Robot %s: Laser pubblicato su %s", robots[ri].id.c_str(), s.topic.c_str());
            }
        }
    }

    tf2_ros::TransformBroadcaster tf_broadcaster;
    ros::Rate rate(10);
    float dt = 0.1;

    ROS_INFO("Simulation started. Usa /move_base_simple/goal per inviare goal a tutti i robot.");

    while (ros::ok()) {
        ros::Time now = ros::Time::now();
        ROS_DEBUG("Ciclo simulazione in corso...");

        // --- Pubblica TF dei robot ---
        publishTF(tf_broadcaster, robots);

        // --- Simula LaserScan ---
        for (auto &as : active_sensors) {
            const auto &rob = robots[as.robot_index];

            Isometry2f robot_pose = Isometry2f::Identity();
            robot_pose.translation() = Vector2f(rob.x, rob.y);
            robot_pose.linear() = Eigen::Rotation2Df(rob.alpha).toRotationMatrix();

            Vector2f sensor_offset(rob.radius, 0);
            Vector2f sensor_global_pos = robot_pose * sensor_offset;
            Isometry2f sensor_pose = robot_pose;
            sensor_pose.translation() = sensor_global_pos;

            // QUESTA È LA PARTE CHE DEVI FAR CONTROLLARE 
            float angle_increment = (as.laser_scan->angle_max - as.laser_scan->angle_min) / as.laser_scan->ranges.size();
            
            for (size_t i = 0; i < as.laser_scan->ranges.size(); ++i) {
                float beam_angle = as.laser_scan->angle_min + angle_increment * i;
                Vector2f d(cos(beam_angle), sin(beam_angle));
                d = sensor_pose.linear() * d;

                as.laser_scan->ranges[i] = grid_map->scanRay(sensor_pose.translation(), d, as.laser_scan->range_max);
            }

            ROS_DEBUG("Laser %s del robot %lu aggiornato: primo range %.2f, ultimo range %.2f",
                      as.cfg.topic.c_str(), as.robot_index,
                      as.laser_scan->ranges.front(), as.laser_scan->ranges.back());

            // Pubblica LaserScan
            sensor_msgs::LaserScan scan_msg;
            scan_msg.header.stamp = now;
            scan_msg.header.frame_id = as.frame_id;
            scan_msg.angle_min = as.laser_scan->angle_min;
            scan_msg.angle_max = as.laser_scan->angle_max;
            scan_msg.angle_increment = angle_increment;
            scan_msg.range_min = as.laser_scan->range_min;
            scan_msg.range_max = as.laser_scan->range_max;
            scan_msg.ranges = as.laser_scan->ranges;
            as.pub.publish(scan_msg);
        }

        // --- Aggiorna robot verso goal ---
        for (size_t i = 0; i < robots.size(); ++i) {
            updateRobotControl(&robots[i], &robot_goals[i]);
            ROS_DEBUG("Robot %s: pos=(%.2f, %.2f), theta=%.2f, v_lin=%.2f, v_ang=%.2f",
                      robots[i].id.c_str(), robots[i].x, robots[i].y, robots[i].alpha,
                      robots[i].current_v_lin, robots[i].current_v_ang);

            updateUnicycleKinematics(robots[i], dt, grid_map);
        }

        // --- Ripubblica mappa ---
        map_msg.header.stamp = now;
        map_pub.publish(map_msg);

        // --- Pubblica pose e odometria ---
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
