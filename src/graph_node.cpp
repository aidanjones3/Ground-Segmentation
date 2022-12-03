#include <ros/ros.h>
#include <ros/console.h>
#include "graph_node.h"
#include <iostream>
#include "graph_utilities.h"

namespace graph_node {

    GraphNode::GraphNode() {
        //ROS_ERROR_ONCE("Please provide a point when creating a GraphNode.");
    }

    GraphNode::GraphNode(const pcl::PointXYZI &point, const int ring_num) :
            shot_(point), ring_id_(ring_num) {
        id_ = -1;
    };

    GraphNode::~GraphNode() {};

    void GraphNode::set_normal_by_edges() {
        normal_ += upper_neighbor_.edge_to_node.cross(right_neighbor_.edge_to_node);
        normal_ += right_neighbor_.edge_to_node.cross(lower_neighbor_.edge_to_node);
        normal_ += lower_neighbor_.edge_to_node.cross(left_neighbor_.edge_to_node);
        normal_ += left_neighbor_.edge_to_node.cross(upper_neighbor_.edge_to_node);

        normal_ /= 4;
    }

    void GraphNode::set_edges_from_neighbors() {
        if (upper_neighbor_.node == nullptr && lower_neighbor_.node == nullptr && right_neighbor_.node == nullptr &&
            left_neighbor_.node == nullptr) {
            return;
        }

        upper_neighbor_.edge_to_node = graph_utilities::compute_edge_between_shots(this->get_shot(), upper_neighbor_.node->get_shot());
        right_neighbor_.edge_to_node = graph_utilities::compute_edge_between_shots(this->get_shot(), right_neighbor_.node->get_shot());
        lower_neighbor_.edge_to_node = graph_utilities::compute_edge_between_shots(this->get_shot(), lower_neighbor_.node->get_shot());
        left_neighbor_.edge_to_node = graph_utilities::compute_edge_between_shots(this->get_shot(), left_neighbor_.node->get_shot());

        return;
    }

    void GraphNode::set_upper_neighbor(std::shared_ptr<GraphNode> upper_node) {
        upper_neighbor_.node = upper_node;
    }

    void GraphNode::set_lower_neighbor(std::shared_ptr<GraphNode> lower_node) {
        lower_neighbor_.node = lower_node;
    }

    void GraphNode::set_right_neighbor(std::shared_ptr<GraphNode> right_node) {
        right_neighbor_.node = right_node;
    }

    void GraphNode::set_left_neighbor(std::shared_ptr<GraphNode> left_node) {
        left_neighbor_.node = left_node;
    }

    void GraphNode::remove_neighbor(const int neighbor_index) {}

    Eigen::Vector3d GraphNode::get_normal() {
        return normal_;
    }

    std::vector<Eigen::Vector3d> GraphNode::get_edges() {
        std::vector<Eigen::Vector3d> edges{upper_neighbor_.edge_to_node, right_neighbor_.edge_to_node,
                                           lower_neighbor_.edge_to_node, left_neighbor_.edge_to_node};
        return edges;
    };

    int GraphNode::get_laser_id() const {
        return ring_id_;
    }

    pcl::PointXYZI GraphNode::get_shot() const {
        return shot_;
    };

}


