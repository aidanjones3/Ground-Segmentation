#include <ros/ros.h>
#include <ros/console.h>
#include "undirected_graph.h"
#include <iostream>

UndirectedGraph::UndirectedGraph(){
    // ROS_WARN_ONCE("The undirected graph created does not contain any nodes.");
};

UndirectedGraph::UndirectedGraph(const sensor_msgs::PointCloud2 &msg){
    // Convert to PCL point cloud first
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    pcl::fromROSMsg(msg, pcl_cloud);
    for(int i = 0; i < pcl_cloud.points.size(); ++i){
        GraphNode node(pcl_cloud.points[i]);
        graph_.emplace(-1, node);
    }

};

UndirectedGraph::~UndirectedGraph(){};