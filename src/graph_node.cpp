#include <ros/ros.h>
#include <ros/console.h>
#include "graph_node.h"
#include <iostream>

GraphNode::GraphNode() {
    //ROS_ERROR_ONCE("Please provide a point when creating a GraphNode.");
}

GraphNode::GraphNode(const pcl::PointXYZI &point, const int ring_num) :
        shot_(point), ring_id_(ring_num) {
    id_ = -1;
};

GraphNode::~GraphNode() {};

void GraphNode::compute_normal_by_neighbors() {
    // Check size
    if (edges_.size() < 2) {
        ROS_ERROR("Node does not contain enough edges to compute a normal.");
    }

    for (int i = 0; i < edges_.size(); ++i) {
        if (i == edges_.size() - 1) {
            normal_ += edges_[i].cross(edges_[0]);
        } else {
            normal_ += edges_[i].cross(edges_[i + 1]);
        }
    }

    normal_ /= edges_.size();
}

void GraphNode::set_neighbors(const std::vector <Eigen::Vector3d> &neighbors) {
    edges_ = neighbors;
}

void GraphNode::remove_neighbor(const int neighbor_index) {}

Eigen::Vector3d GraphNode::get_normal() {
    return normal_;
}

std::vector<Eigen::Vector3d> GraphNode::get_edges(){
    return edges_;
};

int GraphNode::get_laser_id(){
    return ring_id_;
}

pcl::PointXYZI GraphNode::get_shot(){
    return shot_;
};


