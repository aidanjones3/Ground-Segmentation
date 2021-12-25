#include <ros/ros.h>
#include <ros/console.h>
#include "graph_node.h"
#include <gtest/gtest.h>

std::vector<GraphNode> create_graph_nodes(){
	std::vector<GraphNode> nodes;
	return nodes;
}

std::vector<Eigen::Vector3d> create_nominal_edges(){
	const Eigen::Vector3d pos_azimuth = Eigen::Vector3d::UnitX();
	const Eigen::Vector3d neg_azimuth = -Eigen::Vector3d::UnitX();
	const Eigen::Vector3d pos_elevation = Eigen::Vector3d::UnitY();
	const Eigen::Vector3d neg_elevation = -Eigen::Vector3d::UnitY();

	const std::vector<Eigen::Vector3d> edges{pos_azimuth, pos_elevation, neg_azimuth, neg_elevation};
	return edges;
}

TEST(ComputeNormal, failsWithInsufficientEdges){
	const auto nodes = create_graph_nodes();
	EXPECT_EQ(nodes.size(), 0);
}

TEST(ComputeNormal, nominalEdgesCreatesNominalNormal){
	// SETUP
	const Eigen::Vector3d expected_normal = Eigen::Vector3d::UnitZ();
	GraphNode node;
	const auto edges = create_nominal_edges();
	node.set_neighbors(edges);

	// ACTION
	node.compute_normal_by_neighbors();
	const auto normal = node.get_normal();
	
	// VERIFICATION
	EXPECT_EQ(normal.x(), expected_normal.x());
	EXPECT_EQ(normal.y(), expected_normal.y());
	EXPECT_EQ(normal.z(), expected_normal.z());
	
}

int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
