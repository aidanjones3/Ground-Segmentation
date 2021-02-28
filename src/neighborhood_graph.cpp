#include <ros/ros.h>
#include <ros/console.h>
#include "neighborhood_graph.h"
#include <iostream>

GraphNode::GraphNode(){
	//ROS_ERROR_ONCE("Please provide a point when creating a GraphNode.");
}

GraphNode::GraphNode(const pcl::PointXYZ &point) : shot_(point){};

GraphNode::~GraphNode(){};

void GraphNode::compute_normal_by_neighbors(){
	// Check size
	if(edges_.size() < 2){
		ROS_ERROR("Node does not contain enough edges to compute a normal.");
	}
	
}

void GraphNode::set_neighbors(const std::vector<Eigen::Vector3d> &neighbors){}

void GraphNode::remove_neighbor(const int neighbor_index){}

