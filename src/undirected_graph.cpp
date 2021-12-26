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
    pcl::PointCloud <pcl::PointXYZI> pcl_cloud;
    pcl::fromROSMsg(msg, pcl_cloud);

    // Deallocating matrix safely and setting new size
    graph_.resize(0,0);
    graph_.resize(msg.width, 1);

    // Create and add nodes to the undirected graph.
    for (int i = 0; i < pcl_cloud.points.size(); ++i) {
        GraphNode node(pcl_cloud.points[i]);
        graph_(i, 0) = node;
    }
};

UndirectedGraph::~UndirectedGraph() {};

size_t UndirectedGraph::get_graph_size(){
    return graph_.size();
}