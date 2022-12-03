#include <ros/ros.h>
#include <ros/console.h>
#include "graph_node.h"
#include <gtest/gtest.h>

std::vector<graph_node::GraphNode> create_graph_nodes(){
	std::vector<graph_node::GraphNode> nodes;
	return nodes;
}

graph_node::GraphNode create_node_from_location(double x, double y, double z) {
    pcl::PointXYZI point;
    point.x = x;
    point.y = y;
    point.z = z;
    point.intensity = 0;
    return graph_node::GraphNode(point, 0);
}



std::vector<Eigen::Vector3d> create_nominal_edges(){
	const Eigen::Vector3d pos_azimuth = Eigen::Vector3d::UnitX();
	const Eigen::Vector3d neg_azimuth = -Eigen::Vector3d::UnitX();
	const Eigen::Vector3d pos_elevation = Eigen::Vector3d::UnitY();
	const Eigen::Vector3d neg_elevation = -Eigen::Vector3d::UnitY();

	const std::vector<Eigen::Vector3d> edges{pos_azimuth, pos_elevation, neg_azimuth, neg_elevation};
	return edges;
}

//EdgePair create_nominal_vertical_neighbors()

TEST(ComputeNormal, failsWithInsufficientEdges){
	const auto nodes = create_graph_nodes();
	EXPECT_EQ(nodes.size(), 0);
}

TEST(ComputeNormal, nominalEdgesCreatesNominalNormal){
	// SETUP
	const Eigen::Vector3d expected_normal = -Eigen::Vector3d::UnitZ();
	graph_node::GraphNode original_node = create_node_from_location(0, 0, 0);
    const auto upper_node = create_node_from_location(0, 1, 0);
    const auto lower_node = create_node_from_location(0, -1, 0);
    const auto right_node = create_node_from_location(1, 0, 0);
    const auto left_node = create_node_from_location(-1, 0, 0);
    original_node.set_upper_neighbor(std::make_shared<graph_node::GraphNode>(upper_node));
    original_node.set_lower_neighbor(std::make_shared<graph_node::GraphNode>(lower_node));
    original_node.set_left_neighbor(std::make_shared<graph_node::GraphNode>(left_node));
    original_node.set_right_neighbor(std::make_shared<graph_node::GraphNode>(right_node));

	// ACTION
    original_node.set_edges_from_neighbors();
	original_node.set_normal_by_edges();
	const auto normal = original_node.get_normal();

	// VERIFICATION
	EXPECT_NEAR(normal.x(), expected_normal.x(), 0.001);
	EXPECT_NEAR(normal.y(), expected_normal.y(), 0.001);
	EXPECT_NEAR(normal.z(), expected_normal.z(), 0.001);

}

int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
	return RUN_ALL_TESTS();
}
