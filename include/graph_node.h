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

class GraphNode {
public:
    GraphNode();

    GraphNode(const pcl::PointXYZI &point, const int ring_num);

    ~GraphNode();

    void compute_normal_by_neighbors();

    void set_vertical_neighbors(const std::pair <GraphNode, GraphNode> &vert_neighbors);

    void set_horizontal_neighbors(const std::pair <GraphNode, GraphNode> &horz_neighbors);

    void remove_neighbor(const int neighbor_index);

    Eigen::Vector3d get_normal();

    std::vector <GraphNode> get_edges();

    int get_laser_id() const;

    pcl::PointXYZI get_shot() const;

private:
    pcl::PointXYZI shot_;
    std::vector <GraphNode> edges_;
    Eigen::Vector3d normal_;
    int id_;
    int ring_id_;
};
