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


namespace graph_node {

    class GraphNode {

    struct EdgeNode {
        std::shared_ptr<GraphNode> node;
        Eigen::Vector3d edge_to_node;
    };

    public:
        GraphNode();

        GraphNode(const pcl::PointXYZI &point, int ring_num);

        ~GraphNode();

        void set_normal_by_edges();

        void set_edges_from_neighbors();

        void set_upper_neighbor(std::shared_ptr<GraphNode> upper_node);

        void set_lower_neighbor(std::shared_ptr<GraphNode> lower_node);

        void set_right_neighbor(std::shared_ptr<GraphNode> right_node);

        void set_left_neighbor(std::shared_ptr<GraphNode> left_node);

        void remove_neighbor(int neighbor_index);

        Eigen::Vector3d get_normal();

        std::vector<Eigen::Vector3d> get_edges();

        int get_laser_id() const;

        pcl::PointXYZI get_shot() const;

    private:
        pcl::PointXYZI shot_;
        EdgeNode upper_neighbor_;
        EdgeNode lower_neighbor_;
        EdgeNode right_neighbor_;
        EdgeNode left_neighbor_;
        Eigen::Vector3d normal_;
        int id_;
        int ring_id_;
    };

} // namespace




