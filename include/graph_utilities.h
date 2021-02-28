#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <sstream>
#include <eigen3/Eigen/Dense>

namespace graph_utilities{
	
	void create_data_frame();
	
	void find_vertical_edges();

	void find_horizontal_edges();


};
