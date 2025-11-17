#pragma once

#include "grid_map.h"
// #include "isometry_2.h"
// #include "precision.h"

struct LaserScan {
  float range_min = 0.1, range_max = 10, angle_min = -M_PI / 2,
         angle_max = M_PI / 2;
  int ranges_num = 0;
  // float* ranges = nullptr;
  std::vector<float> ranges;

  LaserScan(float range_min = 0.1, float range_max = 10,
            float angle_min = -M_PI / 2, float angle_max = M_PI / 2,
            int ranges_num = 180);

  
};