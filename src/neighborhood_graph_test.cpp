#include <ros/ros.h>
#include <ros/console.h>
#include "neighborhood_graph.h"
#include <gtest/gtest.h>

std::vector<GraphNode> create_graph_nodes(){
	std::vector<GraphNode> nodes;
	return nodes;
}

TEST(ComputeNormal, failsWithInsufficientEdges){
	const auto nodes = create_graph_nodes();
	EXPECT_EQ(nodes.size(), 0);
}

int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
