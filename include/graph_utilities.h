#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include "undirected_graph.h"

namespace graph_utilities {

    void create_undirected_graph();

    // Given an input graph, find all of its vertical edges. Its vertical edges are defined as
    // the nodes with the closest yaw angle in the laser id above and below the current laser id.
    // To find the vertical neighbors, we iterate through each of the nodes of the graph. If they
    // do not have any edges set, we look for that node's neighbors.
    std::pair <GraphNode, GraphNode>
    find_vertical_neighbors(Eigen::Matrix<GraphNode, Eigen::Dynamic, Eigen::Dynamic graph) {
        int laser_id = -1;
        pcl::PointXYZI shot;
        for(auto node : graph.reshaped()){
            if(node.get_edges().size() == 0){
                laser_id = node.get_laser_id();
                shot = node.get_shot();
            }
        }

    };

    // Given an input graph, find all of its horizontal neighbors. Its horizontal neighbors are defined
    // as the nodes with the closest y-coordinate measurement to the left and right of the current node
    // that have the same laser id. To find the horizontal neighbors, we iterate through each of the nodes
    // of the graph. If they have only 2 edges set, we look for that node's horizontal neighbors.
    std::pair <GraphNode, GraphNode>
    find_horizontal_neighbors(Eigen::Matrix<GraphNode, Eigen::Dynamic, Eigen::Dynamic graph) {


    };


};
