#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <sstream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <eigen3/Eigen/Dense>
#include <unordered_map>
#include <map>
#include "graph_node.h"

using namespace graph_node;


class UndirectedGraph {
public:
    UndirectedGraph();

    UndirectedGraph(const sensor_msgs::PointCloud2 &msg);

    ~UndirectedGraph();

    size_t get_graph_size();

    Eigen::Matrix<graph_node::GraphNode, Eigen::Dynamic, Eigen::Dynamic> get_graph();

    // Given an input graph, do the following for every node within the graph:
    //      1. Find all the neighbors, both vertical and horizontal.
    //      2. Using each node's four neighbors, compute the edges of each node.
    //      3. Using the node's edges, compute the normal.
    // @input Eigen::Matrix<GraphNode, Eigen::Dynamic, Eigen::Dynamic> graph: The undirected graph
    // @returns success upon creating a undirected graph where all of the neighbors, edges and normals are set for each node.
    void create_undirected_graph();


    // Given an input graph and input node, find all of its horizontal neighbors. Its horizontal neighbors are defined
    // as the nodes with the closest y-coordinate measurement to the left and right of the current node
    // that have the same laser id. To find the horizontal neighbors, we iterate through each of the nodes
    // of the graph. If they meet the requirements, we set the current node's neighbors.
    std::pair<graph_node::GraphNode, graph_node::GraphNode>
    find_horizontal_neighbors(graph_node::GraphNode cur_node);

    // Given an input graph and input node, find its vertical edges. Its vertical edges are defined as
    // the nodes with the closest yaw angle in the laser id above and below the current laser id.
    // To find the vertical neighbors, we iterate through each of the nodes of the graph. If they
    // meet the requirements, the input node's neighbors are set.
    std::pair<graph_node::GraphNode, graph_node::GraphNode>
    find_vertical_neighbors(graph_node::GraphNode cur_node);


    std::pair<double, double> cartesian_to_polar_coordinates(double x, double y);

private:
    // Graph is stored as an Eigen matrix of GraphNodes.
    Eigen::Matrix<graph_node::GraphNode, Eigen::Dynamic, Eigen::Dynamic> graph_;

    int previous_quadrant_;

    int num_velodyne_lasers_;


};

