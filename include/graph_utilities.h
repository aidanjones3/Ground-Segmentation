#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include "undirected_graph.h"
#include <math.h>

namespace graph_utilities {

    // Converts polar to cartesian coordinates, where theta is in radians.
    std::pair<double, double> cartesian_to_polar_coordinates(double x, double y) {
        double theta_rad = atan(y / x);
        double radius_m = sqrt(x * x + y * y);
        return std::make_pair(theta_rad, radius_m);
    }

    // Given an input graph and input node, find its vertical edges. Its vertical edges are defined as
    // the nodes with the closest yaw angle in the laser id above and below the current laser id.
    // To find the vertical neighbors, we iterate through each of the nodes of the graph. If they
    // meet the requirements, the input node's neighbors are set.
    std::pair <GraphNode, GraphNode>
    find_vertical_neighbors(GraphNode cur_node,
                            const Eigen::Matrix <GraphNode, Eigen::Dynamic, Eigen::Dynamic> &graph) {
        int cur_laser_id = cur_node.get_laser_id();
        pcl::PointXYZI shot = cur_node.get_shot();
        GraphNode top_neighbor;
        GraphNode bottom_neighbor;
        // Case when vertical neighbors are already set.
        if (cur_node.get_edges().size() != 0) {
            return std::make_pair(top_neighbor, bottom_neighbor);
        }
        double cur_yaw_angle = cartesian_to_polar_coordinates(shot.x, shot.y).first;
        double top_yaw_angle_difference = 390.0;
        double bottom_yaw_angle_difference = 390.0;
        // Iterate through every node in the graph to ensure we set the vertical neighbors.
        for (size_t i = 0; i < graph.cols(); ++i) {
            // Case when node belongs to laser directly above current laser.
            if (graph(i, 0).get_laser_id() - cur_laser_id == 1) {
                pcl::PointXYZI potential_shot = graph(i, 0).get_shot();
                double yaw_angle = cartesian_to_polar_coordinates(potential_shot.x, potential_shot.y).first;
                if (abs(yaw_angle - cur_yaw_angle) < top_yaw_angle_difference) {
                    top_neighbor = graph(i, 0);
                    top_yaw_angle_difference = abs(yaw_angle - cur_yaw_angle);
                }
                // Case when node belongs to laser directly below current laser.
            } else if (graph(i, 0).get_laser_id() - cur_laser_id == -1) {
                pcl::PointXYZI potential_shot = graph(i, 0).get_shot();
                double yaw_angle = cartesian_to_polar_coordinates(potential_shot.x, potential_shot.y).first;
                if (abs(yaw_angle - cur_yaw_angle) < bottom_yaw_angle_difference) {
                    bottom_neighbor = graph(i, 0);
                    bottom_yaw_angle_difference = abs(yaw_angle - cur_yaw_angle);
                }
            } else {
                continue;
            }
        }
        return std::make_pair(top_neighbor, bottom_neighbor);
    };

    // Given an input graph and input node, find all of its horizontal neighbors. Its horizontal neighbors are defined
    // as the nodes with the closest y-coordinate measurement to the left and right of the current node
    // that have the same laser id. To find the horizontal neighbors, we iterate through each of the nodes
    // of the graph. If they meet the requirements, we set the current node's neighbors.
    std::pair <GraphNode, GraphNode>
    find_horizontal_neighbors(GraphNode cur_node,
                              const Eigen::Matrix <GraphNode, Eigen::Dynamic, Eigen::Dynamic> &graph) {
        int cur_laser_id = cur_node.get_laser_id();
        pcl::PointXYZI shot = cur_node.get_shot();
        GraphNode left_neighbor;
        GraphNode right_neighbor;
        // Case when horizontal neighbors are already set.
        if (cur_node.get_edges().size() == 4) {
            return std::make_pair(left_neighbor, right_neighbor);
        }
        double cur_y_coordinate = shot.y;
        double left_y_delta = -300.0;
        double right_y_delta = 300.0;
        // Iterate through every node in the graph to ensure we set the vertical neighbors.
        for (size_t i = 0; i < graph.cols(); ++i) {
            // Case when node belongs to different laser id.
            if (graph(i, 0).get_laser_id() != cur_laser_id) {
                continue;
                // Case when node belongs to same laser id, we check it.
            } else {
                pcl::PointXYZI potential_shot = graph(i, 0).get_shot();
                // Case when node is to the left and closer than previous left neighbor.
                //
                // left_neighbor ------------ cur_node
                //                    |
                //                    |
                //            new_left_neighbor
                if (potential_shot.y - cur_y_coordinate > left_y_delta) {
                    left_neighbor = graph(i, 0);
                    left_y_delta = potential_shot.y - cur_y_coordinate;
                    // Case when node is to the right and closer than previous right neighbor.
                    //
                    // cur_node ------------ right_neighbor
                    //               |
                    //               |
                    //      new_right_neighbor
                } else if (potential_shot.y - cur_y_coordinate < right_y_delta) {
                    right_neighbor = graph(i, 0);
                    right_y_delta = potential_shot.y - cur_y_coordinate;
                } else {
                    continue;
                }
            }
        }
        return std::make_pair(left_neighbor, right_neighbor);

    };

    // Given an input graph, do the following for every node within the graph:
    //      1. Find all the neighbors, both vertical and horizontal.
    //      2. Using each node's four neighbors, compute the edges of each node.
    //      3. Using the node's edges, compute the normal.
    // @input Eigen::Matrix<GraphNode, Eigen::Dynamic, Eigen::Dynamic> graph: The undirected graph
    // @returns success upon creating a undirected graph where all of the neighbors, edges and normals are set for each node.
    Eigen::Matrix <GraphNode, Eigen::Dynamic, Eigen::Dynamic>
    create_undirected_graph(Eigen::Matrix <GraphNode, Eigen::Dynamic, Eigen::Dynamic> graph) {
        for (size_t i = 0; i < graph.rows(); ++i) {
            for (size_t j = 0; j < graph.cols(); ++j) {
                auto cur_node = graph(i, j);
                auto vertical_neighbors = find_vertical_neighbors(cur_node, graph);
                cur_node.set_vertical_neighbors(vertical_neighbors);
                auto horz_neighbors = find_horizontal_neighbors(cur_node, graph);
                cur_node.set_horizontal_neighbors(horz_neighbors);
            }
        }
        return graph;
    };


};
