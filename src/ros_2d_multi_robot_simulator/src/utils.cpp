#include "utils.h"

// Include mancanti
#include <yaml-cpp/yaml.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <cmath>
#include <algorithm> // per std::max e std::min

// --- PARSE YAML ---
void parseYAML(const std::string &file,
               MapConfig &map_cfg,
               std::vector<RobotConfig> &robots) {
    YAML::Node cfg = YAML::LoadFile(file);

    // --- Mappa ---
    YAML::Node map_node = cfg["environment"]["map"];
    map_cfg.width = map_node["width"].as<int>();
    map_cfg.height = map_node["height"].as<int>();
    map_cfg.resolution = map_node["resolution"].as<float>();
    map_cfg.origin_x = map_node["origin_x"].as<float>();
    map_cfg.origin_y = map_node["origin_y"].as<float>();

    // Ostacoli
    map_cfg.obstacles.clear();
    if (map_node["obstacles"].IsDefined()) {
        for (const auto &obs : map_node["obstacles"]) {
            if (obs.size() == 4)
                map_cfg.obstacles.push_back({
                    obs[0].as<float>(),
                    obs[1].as<float>(),
                    obs[2].as<float>(),
                    obs[3].as<float>()
                });
        }
    }

    // --- Robot ---
    robots.clear();
    if (cfg["robots"].IsDefined()) {
        for (const auto &r : cfg["robots"]) {
            RobotConfig rc;
            rc.id = r["id"].as<std::string>();
            rc.frame_id = r["frame_id"].as<std::string>();
            rc.radius = r["radius"].as<float>();
            rc.x = r["start_position"]["x"].as<float>();
            rc.y = r["start_position"]["y"].as<float>();
            rc.alpha = r["start_position"]["alpha"].as<float>();
            rc.v_lin = r["max_velocity"]["linear"].as<float>();
            rc.v_ang = r["max_velocity"]["angular"].as<float>();

            // --- Sensori ---
            rc.sensors.clear();
            if (r["devices"].IsDefined()) {
                for (const auto &s : r["devices"]) {
                    SensorConfig sc;
                    sc.type = s["type"].as<std::string>();
                    sc.frame_id = s["frame_id"].as<std::string>();
                    sc.topic = s["topic"].as<std::string>();
                    sc.beams = s["beams"].as<int>();
                    sc.range_min = s["range"]["min"].as<float>();
                    sc.range_max = s["range"]["max"].as<float>();
                    sc.angle_min = s["angle_min"] ? s["angle_min"].as<float>() : -M_PI;
                    sc.angle_max = s["angle_max"] ? s["angle_max"].as<float>() : M_PI;
                    rc.sensors.push_back(sc);
                }
            }

            robots.push_back(rc);
        }
    }
}

// --- CREA OCCUPANCY GRID ---
nav_msgs::OccupancyGrid createMap(const MapConfig &map_cfg) {
    nav_msgs::OccupancyGrid map_msg;
    map_msg.header.frame_id = "map";
    map_msg.info.width = map_cfg.width;
    map_msg.info.height = map_cfg.height;
    map_msg.info.resolution = map_cfg.resolution;
    map_msg.info.origin.position.x = map_cfg.origin_x;
    map_msg.info.origin.position.y = map_cfg.origin_y;
    map_msg.info.origin.position.z = 0.0;
    map_msg.info.origin.orientation.w = 1.0;

    map_msg.data.resize(map_cfg.width * map_cfg.height, 0);

    // Inserisci ostacoli
    for (const auto &obs : map_cfg.obstacles) {
        int col_start = (obs[0] - map_cfg.origin_x) / map_cfg.resolution;
        int row_start = (obs[1] - map_cfg.origin_y) / map_cfg.resolution;
        int col_end   = (obs[2] - map_cfg.origin_x) / map_cfg.resolution;
        int row_end   = (obs[3] - map_cfg.origin_y) / map_cfg.resolution;

        for (int r = row_start; r <= row_end; ++r) {
            for (int c = col_start; c <= col_end; ++c) {
                if (r >=0 && r < map_cfg.height && c >=0 && c < map_cfg.width)
                    map_msg.data[r * map_cfg.width + c] = 100;
            }
        }
    }

    return map_msg;
}

// --- CREA GRID MAP da OccupancyGrid ---
std::shared_ptr<GridMap> createGridMap(const nav_msgs::OccupancyGrid& map_msg) {
    auto grid_map = std::make_shared<GridMap>(map_msg.info.resolution, 
                                               map_msg.info.height, 
                                               map_msg.info.width);
    
    Vector2f origin(map_msg.info.origin.position.x, map_msg.info.origin.position.y);
    grid_map->reset(origin, map_msg.info.resolution);
    
    for (int r = 0; r < grid_map->rows; ++r) {
        for (int c = 0; c < grid_map->cols; ++c) {
            int idx = r * grid_map->cols + c;
            grid_map->cells[idx] = (map_msg.data[idx] > 50) ? 0 : 255;
        }
    }
    
    return grid_map;
}

// --- PUBBLICA TF ---
void publishTF(tf2_ros::TransformBroadcaster &tf_broadcaster, 
               const std::vector<RobotConfig> &robots) {
    ros::Time now = ros::Time::now();

    geometry_msgs::TransformStamped map_to_odom;
    map_to_odom.header.stamp = now;
    map_to_odom.header.frame_id = "map";
    map_to_odom.child_frame_id = "odom";
    map_to_odom.transform.translation.x = 0;
    map_to_odom.transform.translation.y = 0;
    map_to_odom.transform.translation.z = 0;
    map_to_odom.transform.rotation.w = 1.0;
    tf_broadcaster.sendTransform(map_to_odom);

    for (const auto &rc : robots) {
        geometry_msgs::TransformStamped odom_to_robot;
        odom_to_robot.header.stamp = now;
        odom_to_robot.header.frame_id = "odom";
        odom_to_robot.child_frame_id = rc.frame_id;
        odom_to_robot.transform.translation.x = rc.x;
        odom_to_robot.transform.translation.y = rc.y;
        odom_to_robot.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, rc.alpha);
        odom_to_robot.transform.rotation.x = q.x();
        odom_to_robot.transform.rotation.y = q.y();
        odom_to_robot.transform.rotation.z = q.z();
        odom_to_robot.transform.rotation.w = q.w();
        tf_broadcaster.sendTransform(odom_to_robot);

        for (const auto &s : rc.sensors) {
            geometry_msgs::TransformStamped robot_to_sensor;
            robot_to_sensor.header.stamp = now;
            robot_to_sensor.header.frame_id = rc.frame_id;
            robot_to_sensor.child_frame_id = s.frame_id;
            robot_to_sensor.transform.translation.x = rc.radius;
            robot_to_sensor.transform.translation.y = 0.0;
            robot_to_sensor.transform.translation.z = 0.0;

            tf2::Quaternion q_sensor;
            q_sensor.setRPY(0, 0, 0);
            robot_to_sensor.transform.rotation.x = q_sensor.x();
            robot_to_sensor.transform.rotation.y = q_sensor.y();
            robot_to_sensor.transform.rotation.z = q_sensor.z();
            robot_to_sensor.transform.rotation.w = q_sensor.w();
            tf_broadcaster.sendTransform(robot_to_sensor);
        }
    }
}

// --- AGGIORNA CINEMATICA UNICYCLE ---
void updateUnicycleKinematics(RobotConfig &robot, float dt, const std::shared_ptr<GridMap>& grid_map) {
    float v_lin = std::max(-robot.v_lin, std::min(robot.v_lin, robot.current_v_lin));
    float v_ang = std::max(-robot.v_ang, std::min(robot.v_ang, robot.current_v_ang));

    float new_x = robot.x + v_lin * cos(robot.alpha) * dt;
    float new_y = robot.y + v_lin * sin(robot.alpha) * dt;
    float new_alpha = robot.alpha + v_ang * dt;

    while (new_alpha > M_PI) new_alpha -= 2.0 * M_PI;
    while (new_alpha < -M_PI) new_alpha += 2.0 * M_PI;

    float map_origin_x = grid_map->origin().x();
    float map_origin_y = grid_map->origin().y();
    float resolution = grid_map->resolution();
    int cols = grid_map->cols;

    bool collision_detected = false;
    const float SAFETY_MARGIN = 0.5f;
    int cell_radius = (int)std::ceil((robot.radius + SAFETY_MARGIN) / resolution);

    int center_col = (int)((new_x - map_origin_x) / resolution);
    int center_row = (int)((new_y - map_origin_y) / resolution);

    for (int dr = -cell_radius; dr <= cell_radius; ++dr) {
        for (int dc = -cell_radius; dc <= cell_radius; ++dc) {
            int row = center_row + dr;
            int col = center_col + dc;

            if (row < 0 || row >= grid_map->rows || col < 0 || col >= grid_map->cols) {
                collision_detected = true;
                ROS_ERROR_THROTTLE(1.0, "Robot %s ha tentato di uscire dai limiti della mappa.", robot.id.c_str());
                break;
            }

            float cell_x = map_origin_x + (col + 0.5f) * resolution;
            float cell_y = map_origin_y + (row + 0.5f) * resolution;
            float dist = std::sqrt((cell_x - new_x)*(cell_x - new_x) + (cell_y - new_y)*(cell_y - new_y));

            if (dist <= robot.radius) {
                int index = row * cols + col;
                if (grid_map->cells[index] == 0) {
                    collision_detected = true;
                    ROS_ERROR_THROTTLE(0.5, "Collisione FISICA! Robot %s bloccato in traslazione.", robot.id.c_str());
                    break;
                }
            }
        }
        if (collision_detected) break;
    }

    if (collision_detected) {
        robot.alpha = new_alpha;
        robot.current_v_lin = 0.0f;
        return;
    }

    robot.x = new_x;
    robot.y = new_y;
    robot.alpha = new_alpha;
}
