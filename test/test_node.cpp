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
#include "undirected_graph.h"
#include <visualization_msgs/MarkerArray.h>
#include <tf2_msgs/TFMessage.h>

ros::Publisher edges_pub;
ros::Publisher pc_pub;

void velodyneCallback(const sensor_msgs::PointCloud2 &msg) {
    ROS_INFO("Number of Points in Message: [%f]", static_cast<double>(msg.width));
    //ROS_INFO("Callback Message Size: [%f]", msg)

    // Here we can call the graph utilities function create data frame
    UndirectedGraph graph(msg);
    ROS_INFO("Number of points in graph: [%f]", static_cast<double>(graph.get_graph_size()));

    visualization_msgs::MarkerArray markers = graph.create_marker_array_from_graph(ros::Time(msg.header.stamp));
    ROS_INFO("Number of visualization marker arrays: [%f]", static_cast<double>(markers.markers.size()));
    pc_pub.publish(msg);
    edges_pub.publish(markers);
}

void transformCallback(const tf2_msgs::TFMessage &msg) {

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ground_classifier");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ros::Subscriber sub = nh.subscribe("/kitti/velo/pointcloud", 10, velodyneCallback);
    ros::Subscriber tf_static_sub = nh.subscribe("/tf_static", 10, transformCallback);
    edges_pub = nh.advertise<visualization_msgs::MarkerArray>("/graph_edges", 10);
    pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud", 10);

    ros::spin();

    return 0;
}

