#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <sstream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include "graph_utilities.h"

void velodyneCallback(const sensor_msgs::PointCloud2 &msg) {
    ROS_INFO("Number of Points in Message: [%f]", static_cast<double>(msg.width));
    //ROS_INFO("Callback Message Size: [%f]", msg)

    // Here we can call the graph utilities function create data frame
    UndirectedGraph graph(msg);
    ROS_INFO("Number of points in graph: [%f]", static_cast<double>(graph.get_graph_size()));

    ros::Time begin = ros::Time::now();
    graph_utilities::find_vertical_neighbors(graph.get_graph());
    ros::Time end = ros::Time::now();
    ROS_INFO("Time to find vertical neighbors in current data frame: [%f]",
             static_cast<double>(end.toSec() - begin.toSec()));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ground_classifier");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ros::Subscriber sub = nh.subscribe("/points_raw", 10, velodyneCallback);

    ros::spin();

    return 0;
}

