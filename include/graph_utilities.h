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

    void create_undirected_graph() {

    };

    // Given an input graph, find all of its vertical edges. Its vertical edges are defined as
    // the nodes with the closest yaw angle in the laser id above and below the current laser id.
    // To find the vertical neighbors, we iterate through each of the nodes of the graph. If they
    // do not have any edges set, we look for that node's neighbors.
    void find_vertical_neighbors(Eigen::Matrix <GraphNode, Eigen::Dynamic, Eigen::Dynamic> graph) {
        int cur_laser_id = -1;
        pcl::PointXYZI shot;
        // Iterate through every node in the graph to ensure we set the vertical neighbors.
        for (size_t i = 0; i < graph.cols(); ++i) {
            // If this node hasn't been set yet, we look for its vertical neighbors.
            if (graph(i, 0).get_edges().size() == 0) {
                cur_laser_id = graph(i, 0).get_laser_id();
                shot = graph(i, 0).get_shot();
                GraphNode top_neighbor;
                GraphNode bottom_neighbor;

                // Look for vertical neighbors. Nodes with the closest yaw angle in the laser id above and below.
                double cur_yaw_angle = cartesian_to_polar_coordinates(shot.x, shot.y).first;
                double top_yaw_angle_difference = 390.0;
                double bottom_yaw_angle_difference = 390.0;
                for (size_t j = 0; j < graph.cols(); ++j) {
                    // Case when node belongs to laser directly above current laser.
                    if (j != i && graph(j, 0).get_laser_id() - cur_laser_id == 1) {
                        pcl::PointXYZI potential_shot = graph(j, 0).get_shot();
                        double yaw_angle = cartesian_to_polar_coordinates(potential_shot.x, potential_shot.y).first;
                        if (abs(yaw_angle - cur_yaw_angle) < top_yaw_angle_difference) {
                            top_neighbor = graph(j, 0);
                            top_yaw_angle_difference = abs(yaw_angle - cur_yaw_angle);
                        }
                        // Case when node belongs to laser directly below current laser.
                    } else if (j != i && graph(j, 0).get_laser_id() - cur_laser_id == -1) {
                        pcl::PointXYZI potential_shot = graph(j, 0).get_shot();
                        double yaw_angle = cartesian_to_polar_coordinates(potential_shot.x, potential_shot.y).first;
                        if (abs(yaw_angle - cur_yaw_angle) < bottom_yaw_angle_difference) {
                            bottom_neighbor = graph(j, 0);
                            bottom_yaw_angle_difference = abs(yaw_angle - cur_yaw_angle);
                        }
                    } else {
                        continue;
                    }
                }

            } else {
                continue;
            }

        }
    };

    // Given an input graph, find all of its horizontal neighbors. Its horizontal neighbors are defined
    // as the nodes with the closest y-coordinate measurement to the left and right of the current node
    // that have the same laser id. To find the horizontal neighbors, we iterate through each of the nodes
    // of the graph. If they have only 2 edges set, we look for that node's horizontal neighbors.
    std::pair <GraphNode, GraphNode>
    find_horizontal_neighbors(Eigen::Matrix <GraphNode, Eigen::Dynamic, Eigen::Dynamic> graph) {
        return std::make_pair(GraphNode(), GraphNode());


    };


};
