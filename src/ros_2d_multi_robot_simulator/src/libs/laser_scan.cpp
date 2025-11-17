#include "laser_scan.h"

LaserScan::LaserScan(float range_min, float range_max, float angle_min,
                     float angle_max, int ranges_num) {
  this->range_min = range_min;
  this->range_max = range_max;
  this->angle_min = angle_min;
  this->angle_max = angle_max;
  this->ranges_num = ranges_num;

  // ranges = new float[ranges_num];
  // for (int i = 0; i < ranges_num; ++i) ranges[i] = range_max;
  ranges.resize(ranges_num);
  std::fill(ranges.begin(), ranges.end(), range_max);
}

