#include <ros/ros.h>
#include <ros/console.h>
#include "undirected_graph.h"
#include <iostream>
#include "undirected_graph_utilities.h"

UndirectedGraph::UndirectedGraph() {
    // ROS_WARN_ONCE("The undirected graph created does not contain any nodes.");
};

UndirectedGraph::UndirectedGraph(const sensor_msgs::PointCloud2 &msg) {
    if (msg.width == 0) {
        ROS_ERROR_ONCE("Input point cloud for undirected graph does not contain any points.");
    }

    /*
     * Currently, ring data is not available in synced/rectified data. See: https://github.com/tomas789/kitti2bag/issues/27#issuecomment-538026684
     * However, we may be able to recreate it via: https://github.com/VincentCheungM/Run_based_segmentation/issues/3
     *
     */
    //
    // Convert to PCL point cloud first
    pcl::PCLPointCloud2 pcl_cloud;
    pcl_conversions::toPCL(msg, pcl_cloud);

    // Deallocating matrix safely and setting new size
    graph_.resize(0,0);
    graph_.resize(msg.width, 1);

    // Create and add nodes to the undirected graph.
    previous_quadrant_ = 0;
    int ring_id = 0;
    int num_velodyne_lasers_ = 64;
    for (int i = 0; i < pcl_cloud.height * pcl_cloud.width; ++i) {
        pcl::PointXYZI point;
        // Getting information from pcl cloud and creating custom point from it
        const auto x = static_cast<float>(pcl_cloud.data[i * pcl_cloud.point_step + pcl_cloud.fields[0].offset]);
        const auto y = static_cast<float>(pcl_cloud.data[i * pcl_cloud.point_step + pcl_cloud.fields[1].offset]);
        const auto z = static_cast<float>(pcl_cloud.data[i * pcl_cloud.point_step + pcl_cloud.fields[2].offset]);
        const auto intensity = static_cast<float>(pcl_cloud.data[i * pcl_cloud.point_step + pcl_cloud.fields[3].offset]);
        // const int laser_id = static_cast<int>(pcl_cloud.data[i * pcl_cloud.point_step + pcl_cloud.fields[4].offset]);
        point.x = x;
        point.y = y;
        point.z = z;
        point.intensity = intensity;

        const auto quadrant_of_point = undirected_graph_utilities::get_quadrant_from_point(x, y);
        if(quadrant_of_point == 4 && previous_quadrant_ == 1 && ring_id < num_velodyne_lasers_ - 1){
            ring_id += 1;
        }

        // Create new graph node with custom point and add it to the undirected graph
        graph_node::GraphNode node(point, ring_id);
        graph_(i, 0) = node;
        previous_quadrant_ = quadrant_of_point;
    }

    create_undirected_graph();

};

UndirectedGraph::~UndirectedGraph() {};

size_t UndirectedGraph::get_graph_size(){
    return graph_.size();
}

Eigen::Matrix<GraphNode, Eigen::Dynamic, Eigen::Dynamic> UndirectedGraph::get_graph(){
    return graph_;
};

// Converts polar to cartesian coordinates, where theta is in radians.
std::pair<double, double> UndirectedGraph::cartesian_to_polar_coordinates(double x, double y) {
    double theta_rad = atan(y / x);
    double radius_m = sqrt(x * x + y * y);
    return std::make_pair(theta_rad, radius_m);
}

// Given an input graph and input node, find its vertical edges. Its vertical edges are defined as
// the nodes with the closest yaw angle in the laser id above and below the current laser id.
// To find the vertical neighbors, we iterate through each of the nodes of the graph. If they
// meet the requirements, the input node's neighbors are set.
std::pair<graph_node::GraphNode, graph_node::GraphNode>
UndirectedGraph::find_vertical_neighbors(graph_node::GraphNode cur_node) {
    int cur_laser_id = cur_node.get_laser_id();
    pcl::PointXYZI shot = cur_node.get_shot();
    graph_node::GraphNode top_neighbor;
    graph_node::GraphNode bottom_neighbor;
    // Case when vertical neighbors are already set.
    if (cur_node.get_edges().size() != 0) {
        return std::make_pair(top_neighbor, bottom_neighbor);
    }
    double cur_yaw_angle = cartesian_to_polar_coordinates(shot.x, shot.y).first;
    double top_yaw_angle_difference = 390.0;
    double bottom_yaw_angle_difference = 390.0;
    // Iterate through every node in the graph to ensure we set the vertical neighbors.
    for (size_t i = 0; i < graph_.cols(); ++i) {
        // Case when node belongs to laser directly above current laser.
        if (graph_(i, 0).get_laser_id() - cur_laser_id == 1) {
            pcl::PointXYZI potential_shot = graph_(i, 0).get_shot();
            double yaw_angle = cartesian_to_polar_coordinates(potential_shot.x, potential_shot.y).first;
            if (abs(yaw_angle - cur_yaw_angle) < top_yaw_angle_difference) {
                top_neighbor = graph_(i, 0);
                top_yaw_angle_difference = abs(yaw_angle - cur_yaw_angle);
            }
            // Case when node belongs to laser directly below current laser.
        } else if (graph_(i, 0).get_laser_id() - cur_laser_id == -1) {
            pcl::PointXYZI potential_shot = graph_(i, 0).get_shot();
            double yaw_angle = cartesian_to_polar_coordinates(potential_shot.x, potential_shot.y).first;
            if (abs(yaw_angle - cur_yaw_angle) < bottom_yaw_angle_difference) {
                bottom_neighbor = graph_(i, 0);
                bottom_yaw_angle_difference = abs(yaw_angle - cur_yaw_angle);
            }
        } else {
            continue;
        }
    }
    return std::make_pair(top_neighbor, bottom_neighbor);
};


std::pair<graph_node::GraphNode, graph_node::GraphNode>
UndirectedGraph::find_horizontal_neighbors(graph_node::GraphNode cur_node) {
    int cur_laser_id = cur_node.get_laser_id();
    pcl::PointXYZI shot = cur_node.get_shot();
    graph_node::GraphNode left_neighbor;
    graph_node::GraphNode right_neighbor;
    // Case when horizontal neighbors are already set.
    if (cur_node.get_edges().size() == 4) {
        return std::make_pair(left_neighbor, right_neighbor);
    }
    double cur_y_coordinate = shot.y;
    double left_y_delta = -300.0;
    double right_y_delta = 300.0;
    // Iterate through every node in the graph to ensure we set the vertical neighbors.
    for (size_t i = 0; i < graph_.cols(); ++i) {
        // Case when node belongs to different laser id.
        if (graph_(i, 0).get_laser_id() != cur_laser_id) {
            continue;
            // Case when node belongs to same laser id, we check it.
        } else {
            pcl::PointXYZI potential_shot = graph_(i, 0).get_shot();
            // Case when node is to the left and closer than previous left neighbor.
            //
            // left_neighbor ------------ cur_node
            //                    |
            //                    |
            //            new_left_neighbor
            if (potential_shot.y - cur_y_coordinate > left_y_delta) {
                left_neighbor = graph_(i, 0);
                left_y_delta = potential_shot.y - cur_y_coordinate;
                // Case when node is to the right and closer than previous right neighbor.
                //
                // cur_node ------------ right_neighbor
                //               |
                //               |
                //      new_right_neighbor
            } else if (potential_shot.y - cur_y_coordinate < right_y_delta) {
                right_neighbor = graph_(i, 0);
                right_y_delta = potential_shot.y - cur_y_coordinate;
            } else {
                continue;
            }
        }
    }
    return std::make_pair(left_neighbor, right_neighbor);

};


void UndirectedGraph::create_undirected_graph() {
    ros::Time begin = ros::Time::now();
    for (size_t i = 0; i < graph_.rows(); ++i) {
        for (size_t j = 0; j < graph_.cols(); ++j) {
            auto cur_node = graph_(i, j);
            auto vertical_neighbors = find_vertical_neighbors(cur_node);
            cur_node.set_upper_neighbor(std::make_shared<graph_node::GraphNode>(vertical_neighbors.first));
            cur_node.set_lower_neighbor(std::make_shared<graph_node::GraphNode>(vertical_neighbors.second));
            auto horz_neighbors = find_horizontal_neighbors(cur_node);
            cur_node.set_left_neighbor(std::make_shared<graph_node::GraphNode>(horz_neighbors.first));
            cur_node.set_right_neighbor(std::make_shared<graph_node::GraphNode>(horz_neighbors.second));
            graph_(i,j) = cur_node;
        }
    }
    ros::Time end = ros::Time::now();
    ROS_INFO("Time to find neighbors in current data frame: [%f]",
             static_cast<double>(end.toSec() - begin.toSec()));
};