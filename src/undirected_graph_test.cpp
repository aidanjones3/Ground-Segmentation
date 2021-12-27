#include <ros/ros.h>
#include <ros/console.h>
#include "undirected_graph.h"
#include <gtest/gtest.h>

// Local variable constants
const int NUM_POINTS = 100;

sensor_msgs::PointCloud2 create_point_cloud(){
    sensor_msgs::PointCloud2 pc2_msg;
    pcl::PointCloud<pcl::PointXYZI> cloud;
    for(int i = 0; i < NUM_POINTS; ++i){
        const float fr = static_cast<float>(i) / static_cast<float>(NUM_POINTS);
        pcl::PointXYZI pt;
        pt.x = cos(fr * M_PI * 2.0) * 1.0;
        pt.y = sin(fr * M_PI * 2.0) * 1.0;
        pt.z = 0.0;
        pt.intensity = 0.0;
        cloud.points.push_back(pt);
    }
    pcl::toROSMsg(cloud, pc2_msg);
    return pc2_msg;
}


