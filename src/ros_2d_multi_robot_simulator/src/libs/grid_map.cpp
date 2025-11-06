#include "grid_map.h"
using namespace std;

GridMap::GridMap(float resolution_, int rows_, int cols_):
  Grid_<uint8_t>(rows_, cols_){
  reset(Vector2f(0,0), resolution_);
}

float GridMap::scanRay(const Vector2f& origin,
                        const Vector2f& direction,
                        const float max_range) const {
  float range = 0;

  while (range < max_range) {
    Vector2i int_endpoint=world2grid(origin + direction * range).cast<int>();
    
    if (!inside(int_endpoint))
      return max_range;

    if ((*this)(int_endpoint) < 127)
      return range;

    range +=resolution();
  }
  
  return max_range;
}

