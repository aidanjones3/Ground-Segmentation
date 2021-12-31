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



class UndirectedGraph {
 public:
    UndirectedGraph();

    UndirectedGraph(const sensor_msgs::PointCloud2 &msg);

    ~UndirectedGraph();

    size_t get_graph_size();

    Eigen::Matrix<GraphNode, Eigen::Dynamic, Eigen::Dynamic> get_graph();

 private:
    // Graph is stored as an Eigen matrix of GraphNoes.
    Eigen::Matrix<GraphNode, Eigen::Dynamic, Eigen::Dynamic> graph_;


};

