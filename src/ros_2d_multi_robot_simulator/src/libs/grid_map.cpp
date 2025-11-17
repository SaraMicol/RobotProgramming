#include "grid_map.h"
using namespace std;
#include <opencv2/opencv.hpp>  
#include <opencv2/imgcodecs.hpp> 
#include <ros/ros.h>

GridMap::GridMap(float resolution_, int rows_, int cols_)
    : Grid_<uint8_t>(rows_, cols_)
{
    reset(Vector2f(0,0), resolution_);
}

float GridMap::scanRay(const Vector2f& origin,
                       const Vector2f& direction,
                       float max_range) const
{
    Vector2f pos = world2grid(origin);
    Vector2f dir = direction.normalized();
    float dist = 0;

    while (dist < max_range) {
        int r = static_cast<int>(pos.y());
        int c = static_cast<int>(pos.x());
        if (r < 0 || r >= rows || c < 0 || c >= cols)
            break;  // fuori mappa

        if ((*this)(r, c) > 127)
            return dist * resolution();

        pos += dir;
        dist += 1.0f;
    }
    return max_range;
}

bool GridMap::loadFromImage(const char* filename, float resolution)
{
    cv::Mat img = cv::imread(filename, cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
        ROS_ERROR("Impossibile aprire immagine: %s", filename);
        return false;
    }

    // --- IMPORTANTE ---
    // img.rows  = height  (→ rows)
    // img.cols  = width   (→ cols)
    rows = img.rows;
    cols = img.cols;

    // Aggiorna risoluzione e resetta origine
    reset(Vector2f(0, 0), resolution);

    // Ridimensiona griglia
    resize(rows, cols);

    // Copia i pixel nell'occupancy grid
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            uint8_t v = img.at<uint8_t>(r, c);
            (*this)(r, c) = v;
        }
    }

    ROS_INFO("Mappa caricata: rows=%d, cols=%d, resolution=%.3f",
             rows, cols, resolution);

    return true;
}
