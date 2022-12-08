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

    visualization_msgs::Marker create_marker_from_vector(const pcl::PointXYZI &origin_shot, const pcl::PointXYZI &neighbor_shot, ros::Time current_timestamp){
        visualization_msgs::Marker marker;
        marker.points.resize(2);
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.pose.position.x = origin_shot.x;
        marker.pose.position.y = origin_shot.y;
        marker.pose.position.z = origin_shot.z;
        marker.points[0].x = origin_shot.x;
        marker.points[0].y = origin_shot.y;
        marker.points[0].z = origin_shot.z;
        marker.points[1].x = neighbor_shot.x;
        marker.points[1].y = neighbor_shot.y;
        marker.points[1].z = neighbor_shot.z;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.1;
        marker.color.b = 0.0;
        marker.header.frame_id = "velo_link";
        marker.header.stamp = current_timestamp;

        return marker;

    }

}



