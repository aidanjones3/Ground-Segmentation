#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <sstream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/point_cloud2_iterator.h>

void velodyneCallback(const sensor_msgs::PointCloud2 &msg){
	ROS_INFO("Number of Points in Message: [%f]", static_cast<double>(msg.width));
	//ROS_INFO("Callback Message Size: [%f]", msg)

    // Here we can call the graph utilities function create data frame

}

int main(int argc, char **argv)
{
        ros::init(argc, argv, "ground_classifier");

        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

	    ros::Subscriber sub = nh.subscribe("/points_raw", 10, velodyneCallback);

        ros::spin();

        return 0;
}

