#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <sstream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/point_cloud2_iterator.h>

template<N>
class GraphNode {
 public:
	 GraphNode();
	 GraphNode(const pcl::PointXYZ &point);
	 
	 ~GraphNode();

 private:
	 pcl::PointXYZ point;
};
