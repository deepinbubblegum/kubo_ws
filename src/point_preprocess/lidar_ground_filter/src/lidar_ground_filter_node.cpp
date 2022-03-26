#include "ros/ros.h"
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>
#include "sensor_msgs/PointCloud2.h"

class SubscribeProcessPublish
{
private:
    ros::NodeHandle nh;
    ros::Subscriber subscriber;
    ros::Publisher publisher;
  	ros::ServiceClient client;
  	double tolerance;

public:
    SubscribeProcessPublish()
    {
        this->subscriber = this->nh.subscribe<pcl::PCLPointCloud2>(
            "/rs_points",
            5,
            &SubscribeProcessPublish::PointCloudGroundFilter,
            this
        );

        this->publisher = this->nh.advertise<pcl::PCLPointCloud2>("rs_points_ground_filter", 1);
    }

    void PointCloudGroundFilter(const pcl::PCLPointCloud2ConstPtr& cloud_msg)
    {
        // std::cout << "Received lidar measurement with seq ID " << cloud_msg->header.seq << std::endl;

		// define a new container for the data
		pcl::PCLPointCloud2::Ptr cloudVoxel (new pcl::PCLPointCloud2 ());
		
		// define a voxelgrid
		pcl::VoxelGrid<pcl::PCLPointCloud2> voxelGrid;
		// set input to cloud
		voxelGrid.setInputCloud(cloud_msg);
		// set the leaf size (x, y, z)
		voxelGrid.setLeafSize(0.05f, 0.05f, 0.05f);
		// apply the filter to dereferenced cloudVoxel
		voxelGrid.filter(*cloudVoxel);

        pcl::PCLPointCloud2::Ptr floorRemoved (new pcl::PCLPointCloud2 ());
		// define a PassThrough
		pcl::PassThrough<pcl::PCLPointCloud2> pass;

        // set input to cloudVoxel
		pass.setInputCloud(cloudVoxel);
		// filter along z-axis
		pass.setFilterFieldName("z");

        // set z-limits
		pass.setFilterLimits(-0.18, 2.0);
		pass.filter(*floorRemoved);

        // Publish the data
        // this->publisher.publish (*cloudVoxel);
        // this->publisher.publish (cloud_msg);
        this->publisher.publish(*floorRemoved);
    }
};


int main(int argc, char** argv)
{
    // initialise the node
    ros::init(argc, argv, "lidar_ground_filter");

    std::cout << "lidar_ground_filter node initialised" << std::endl;

    // create instance of PublishSubscribe
    SubscribeProcessPublish process;

    // handle ROS communication events
    ros::spin();

    return 0;
}