#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <yaml-cpp/yaml.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <vector>
#include <string>

// --- STRUTTURE DATI ---
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
};

struct MapConfig {
    int width, height;
    float resolution;
    float origin_x, origin_y;
    std::vector<std::vector<float>> obstacles;
};

// --- PARSE YAML ---
void parseYAML(const std::string& file,
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

    map_msg.data.resize(map_cfg.width * map_cfg.height, 0); // libero

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

// --- PUBBLICA TF ---
void publishTF(tf2_ros::TransformBroadcaster &tf_broadcaster, 
               const std::vector<RobotConfig> &robots) {
    ros::Time now = ros::Time::now();

    // map -> odom
    geometry_msgs::TransformStamped map_to_odom;
    map_to_odom.header.stamp = now;
    map_to_odom.header.frame_id = "map";
    map_to_odom.child_frame_id = "odom";
    map_to_odom.transform.translation.x = 0;
    map_to_odom.transform.translation.y = 0;
    map_to_odom.transform.translation.z = 0;
    map_to_odom.transform.rotation.w = 1.0;
    tf_broadcaster.sendTransform(map_to_odom);

    // odom -> robot e robot -> lidar
    for (const auto &rc : robots) {
        // odom -> robot
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

        // robot -> sensori (lidar, ecc.)
        for (const auto &s : rc.sensors) {
            geometry_msgs::TransformStamped robot_to_sensor;
            robot_to_sensor.header.stamp = now;
            robot_to_sensor.header.frame_id = rc.frame_id;
            robot_to_sensor.child_frame_id = s.frame_id;
            robot_to_sensor.transform.translation.x = rc.radius; // davanti al robot
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


// --- STRUTTURA PER SENSORI ATTIVI ---
struct ActiveSensor {
    SensorConfig cfg;
    ros::Publisher pub;
    size_t robot_index;
};
#include <sensor_msgs/LaserScan.h>
#include <cmath>

// --- Funzione di simulazione del LIDAR ---
std::vector<float> simulateLaserScan(const nav_msgs::OccupancyGrid& map,
                                     const SensorConfig& sensor,
                                     float robot_x, float robot_y, float robot_alpha)
{
    std::vector<float> ranges(sensor.beams, sensor.range_max);

    float angle_increment = (sensor.angle_max - sensor.angle_min) / sensor.beams;

    // Scansiona ogni raggio
    for (int i = 0; i < sensor.beams; ++i) {
        float beam_angle = robot_alpha + sensor.angle_min + i * angle_increment;

        // Prova a estendere il raggio fino al massimo range
        for (float r = sensor.range_min; r <= sensor.range_max; r += map.info.resolution) {
            float x = robot_x + r * cos(beam_angle);
            float y = robot_y + r * sin(beam_angle);

            int col = (x - map.info.origin.position.x) / map.info.resolution;
            int row = (y - map.info.origin.position.y) / map.info.resolution;

            // Se fuori mappa, fermati
            if (row < 0 || row >= map.info.height || col < 0 || col >= map.info.width) {
                ranges[i] = r;
                break;
            }

            int idx = row * map.info.width + col;
            if (map.data[idx] > 50) { // muro o ostacolo
                ranges[i] = r;
                break;
            }
        }
    }

    return ranges;
}


// --- MAIN ---
int main(int argc, char** argv) {
    ros::init(argc, argv, "robsim_node");
    ros::NodeHandle nh;

    MapConfig map_cfg;
    std::vector<RobotConfig> robots;

    std::string yaml_file = "/home/lattinone/ros-multi-robot-sim/src/ros_2d_multi_robot_simulator/configs/env1.yaml";
    parseYAML(yaml_file, map_cfg, robots);

   

    auto map_msg = createMap(map_cfg);

    // Publisher mappa e pose
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
    std::vector<ros::Publisher> pose_pubs;
    for (auto &rc : robots)
        pose_pubs.push_back(nh.advertise<geometry_msgs::PoseStamped>("/" + rc.id + "/pose", 1));

    // Publisher lidar
    std::vector<ActiveSensor> active_sensors;
    for (size_t ri = 0; ri < robots.size(); ++ri) {
        for (const auto &s : robots[ri].sensors) {
            if (s.type == "lidar" || s.type == "laser") {
                ActiveSensor as;
                as.cfg = s;
                as.robot_index = ri;
                as.pub = nh.advertise<sensor_msgs::LaserScan>(s.topic, 1);
                active_sensors.push_back(as);
            }
        }
    }

    tf2_ros::TransformBroadcaster tf_broadcaster;

    ros::Rate rate(10);
    while (ros::ok()) {
        ros::Time now = ros::Time::now();
        map_msg.header.stamp = now;
        map_pub.publish(map_msg);

        publishTF(tf_broadcaster, robots);

        // Pose robots
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
        }

        // Simula e pubblica LaserScan
        for (auto &as : active_sensors) {
            const auto &s = as.cfg;
            const auto &rob = robots[as.robot_index];

            sensor_msgs::LaserScan scan;
            scan.header.stamp = now;
            scan.header.frame_id = s.frame_id;
            scan.angle_min = s.angle_min;
            scan.angle_max = s.angle_max;
            scan.angle_increment = (s.angle_max - s.angle_min) / s.beams;
            scan.time_increment = 0.0;
            scan.scan_time = 0.1;
            scan.range_min = s.range_min;
            scan.range_max = s.range_max;

            scan.ranges = simulateLaserScan(map_msg, s, rob.x, rob.y, rob.alpha);
            as.pub.publish(scan);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
