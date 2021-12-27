#include <ros/ros.h>
#include <ros/console.h>
#include "undirected_graph.h"
#include <iostream>

UndirectedGraph::UndirectedGraph() {
    // ROS_WARN_ONCE("The undirected graph created does not contain any nodes.");
};

UndirectedGraph::UndirectedGraph(const sensor_msgs::PointCloud2 &msg) {
    if (msg.width == 0) {
        ROS_ERROR_ONCE("Input point cloud for undirected graph does not contain any points.");
    }
    // Convert to PCL point cloud first
    pcl::PCLPointCloud2 pcl_cloud;
    pcl_conversions::toPCL(msg, pcl_cloud);

    // Deallocating matrix safely and setting new size
    graph_.resize(0,0);
    graph_.resize(msg.width, 1);

    // Create and add nodes to the undirected graph.
    for (int i = 0; i < pcl_cloud.height * pcl_cloud.width; ++i) {
        pcl::PointXYZI point;
        // Getting information from pcl cloud and creating custom point from it
        const double x = static_cast<double>(pcl_cloud.data[i * pcl_cloud.point_step + pcl_cloud.fields[0].offset]);
        const double y = static_cast<double>(pcl_cloud.data[i * pcl_cloud.point_step + pcl_cloud.fields[1].offset]);
        const double z = static_cast<double>(pcl_cloud.data[i * pcl_cloud.point_step + pcl_cloud.fields[2].offset]);
        const double intensity = static_cast<double>(pcl_cloud.data[i * pcl_cloud.point_step + pcl_cloud.fields[3].offset]);
        const int laser_id = static_cast<int>(pcl_cloud.data[i * pcl_cloud.point_step + pcl_cloud.fields[4].offset]);
        point.x = x;
        point.y = y;
        point.z = z;
        point.intensity = intensity;

        // Create new graph node with custom point and add it to the undirected graph
        GraphNode node(point, laser_id);
        graph_(i, 0) = node;

    }
};

UndirectedGraph::~UndirectedGraph() {};

size_t UndirectedGraph::get_graph_size(){
    return graph_.size();
}