#include "laser_scanner.h"
#include <cmath>

LaserScanner::LaserScanner(LaserScan& scn, 
                           Eigen::Isometry2f* robot_pose,
                           GridMap* map,
                           const Eigen::Isometry2f& sensor_offset,
                           float frequency)
    : scan(scn), 
      robot_pose_ptr(robot_pose),
      grid_map(map),
      pose_in_parent(sensor_offset),
      scan_ready(false),
      radius(0.1f),
      counter(0.0f) {
    
    period = 1.0f / frequency;
}

Eigen::Isometry2f LaserScanner::globalPose() const {
    return (*robot_pose_ptr) * pose_in_parent;
}

void LaserScanner::getScan() {
    Eigen::Isometry2f gp = globalPose();
    
    Eigen::Isometry2f rotation = Eigen::Isometry2f::Identity();
    rotation.linear() = gp.linear();
    
    float angle_increment = (scan.angle_max - scan.angle_min) / (scan.ranges.size() - 1);
    
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        float beam_angle = scan.angle_min + angle_increment * i;
        Eigen::Vector2f d(cos(beam_angle), sin(beam_angle));
        d = rotation * d;
        scan.ranges[i] = grid_map->scanRay(gp.translation(), d, scan.range_max);
    }
}

void LaserScanner::tick(float dt) {
    counter += dt;
    scan_ready = false;
    
    if (counter >= period) {
        counter -= period;
        scan_ready = true;
        getScan();
    }
}