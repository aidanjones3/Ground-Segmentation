#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <math.h>

namespace undirected_graph_utilities {

    int get_quadrant_from_point(const double x, const double y) {
        if (x > 0.0 && y >= 0)
            return 1;
        else if (x <= 0 && y > 0) {
            return 2;
        } else if (x < 0 && y <= 0) {
            return 3;
        } else if (x >= 0 && y < 0) {
            return 4;
        }

        return 0;
    }

}



