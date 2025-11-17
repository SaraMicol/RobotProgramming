#ifndef LASER_SCANNER_H
#define LASER_SCANNER_H

#include <Eigen/Geometry>
#include "laser_scan.h"
#include "grid_map.h"

class LaserScanner {
public:
    // Costruttore MODIFICATO per la Soluzione 2
    LaserScanner(LaserScan& scn, 
                 Eigen::Isometry2f* robot_pose,
                 GridMap* map,
                 const Eigen::Isometry2f& sensor_offset, 
                 float frequency);
    
    // Metodi pubblici
    void tick(float dt);
    void getScan();
    
    // Membri pubblici
    LaserScan& scan;
    Eigen::Isometry2f pose_in_parent;
    bool scan_ready;
    float radius;
    
private:
    Eigen::Isometry2f* robot_pose_ptr;
    GridMap* grid_map;
    
    float period;
    float counter;
    
    Eigen::Isometry2f globalPose() const;
};

#endif