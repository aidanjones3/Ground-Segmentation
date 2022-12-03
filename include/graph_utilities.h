#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace graph_utilities {

    Eigen::Vector3d compute_edge_between_shots(pcl::PointXYZI first_shot, pcl::PointXYZI second_shot) {
        Eigen::Vector3d edge(second_shot.x - first_shot.x, second_shot.y - first_shot.y, second_shot.z - first_shot.z);
        return edge;
    }

};
