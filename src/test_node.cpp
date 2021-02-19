#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <sstream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/point_cloud2_iterator.h>

int main(int argc, char **argv)
{
        ros::init(argc, argv, "ground_classifier");

        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        ros::spin();

        return 0;
}

