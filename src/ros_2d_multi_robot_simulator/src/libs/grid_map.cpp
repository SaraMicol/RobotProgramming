#include "grid_map.h"

GridMap::GridMap(float res, int rows_, int cols_)
    : Grid_<uint8_t>(rows_, cols_) // chiama costruttore base
{
    reset(Vector2f(0,0), res);
    // data viene ereditato da Grid_<uint8_t>, usa this->data se protetto
    if constexpr (requires { this->data; }) {
        this->data.resize(this->rows * this->cols);
    }
}

void GridMap::loadFromImage(const char* filename, float res)
{
    cv::Mat img = cv::imread(filename, cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
        ROS_ERROR("Failed to load map image: %s", filename);
        return;
    }

    this->rows = img.rows;
    this->cols = img.cols;
    reset(Vector2f(0,0), res);

    if constexpr (requires { this->data; }) {
        this->data.resize(this->rows * this->cols);

        for (int r = 0; r < this->rows; ++r) {
            for (int c = 0; c < this->cols; ++c) {
                uint8_t value = img.at<uint8_t>(r, c);
                this->data[r * this->cols + c] = (value < 127) ? 100 : 0;
            }
        }
    }

    ROS_INFO("Loaded map from image: %s (%dx%d, resolution %.3f)", filename, this->cols, this->rows, res);
}
