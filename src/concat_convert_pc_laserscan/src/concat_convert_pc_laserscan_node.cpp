#include "std_msgs/String.h"

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "laser_geometry/laser_geometry.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"

#include <tf2_ros/transform_listener.h>
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"

#include <pcl_ros/point_cloud.h>
// #include <pcl/point_cloud.h>
// #include "pcl_conversions/pcl_conversions.h"

float min_ang_, max_ang_, range_min_, range_max_;
bool c1 = false;
bool c2 = false;
std::string frame_id_;
ros::Publisher scan_pub_, cloud_pub_;
sensor_msgs::PointCloud2 cloud_1;
sensor_msgs::PointCloud2 cloud_2;
sensor_msgs::PointCloud2 concatenated_cloud;
sensor_msgs::PointCloud2 cloud_1_out;
sensor_msgs::PointCloud2 cloud_2_out;
sensor_msgs::LaserScan concatenated_scan;
laser_geometry::LaserProjection projector_;
geometry_msgs::TransformStamped cloud_1_transform, cloud_2_transform;

sensor_msgs::LaserScanPtr pointcloud_to_laserscan(sensor_msgs::PointCloud2 *merged_cloud)
{
    sensor_msgs::LaserScanPtr output(new sensor_msgs::LaserScan());
    output->header = merged_cloud->header;
    output->header.frame_id = "base_link";
    output->header.stamp = ros::Time::now();
    output->angle_min = -3.12414;
    // -179 degrees
    output->angle_max = 3.12414;
    // +179 degrees
    output->angle_increment = 0.0087;
    
    output->time_increment = 0.000185185184819;
    output->scan_time = 0.0666666701436;
    output->range_min = 0.1;
    output->range_max = 150.0;
    float min_height_ = -0.15;
    float max_height_ = 2.0;
    float inf = std::numeric_limits<float>::infinity();
    uint32_t ranges_size = std::ceil((output->angle_max - output->angle_min) / output->angle_increment);
    output->ranges.assign(ranges_size, inf);
    output->intensities.assign(ranges_size, 0);
    for (sensor_msgs::PointCloud2ConstIterator<float> it(*merged_cloud, "x"); it != it.end(); ++it)
    {
        const float &x = it[0];
        const float &y = it[1];
        const float &z = it[2];
        const float &intensity = it[3];
        if (std::isnan(x) || std::isnan(y) || std::isnan(z))
        {
            ROS_DEBUG("rejected for nan in point(%f, %f, %f)\n", x, y, z);
            continue;
        }

        double range_sq = y * y + x * x;
        double intensity_sq = intensity;
        double range_min_sq_ = output->range_min * output->range_min;
        if (range_sq < range_min_sq_)
        {
            ROS_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range_sq, range_min_sq_, x, y, z);
            continue;
        }

        
        if (z > max_height_ || z < min_height_)
        {
            //ROS_INFO("reject z = %f",z);
            continue;
        }

        double angle = atan2(y, x);
        if (angle < output->angle_min || angle > output->angle_max)
        {
            ROS_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output->angle_min, output->angle_max);
            continue;
        }
        int index = (angle - output->angle_min) / output->angle_increment;

        if (output->ranges[index] * output->ranges[index] > range_sq)
        {
            output->ranges[index] = sqrt(range_sq);
            output->intensities[index] = intensity_sq;
        }
    }

    return output;
}


void concat_with_pc()
{
    // while (ros::ok())
    // {
        // boost::shared_ptr<sensor_msgs::PointCloud2 const> sharedPtr;
        // sensor_msgs::LaserScan scan_tim_4;
        

        // sharedPtr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/front/rslidar_points", ros::Duration(1));
        // if (sharedPtr == NULL)
        //     //ROS_INFO("No laser messages received");
        // else
        // {
        //     cloud_1 = *sharedPtr;
        //     c1 = true;
        //     //ROS_INFO("Receive1");
        // }
        // sensor_msgs::LaserScan scan_tim_8;

        // sharedPtr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/back/rslidar_points", ros::Duration(1));
        // if (sharedPtr == NULL)
        //     //ROS_INFO("No laser messages received");
        // else
        // {
        //     cloud_2 = *sharedPtr;
        //     c2 = true;
        //     //ROS_INFO("Receive2");
        // }
        //ROS_INFO("3");
        if(c1 == true && c2 == true)
        {
            //ROS_INFO("4");
            cloud_1_transform.header.frame_id = "base_link";
            cloud_2_transform.header.frame_id = "base_link";
            cloud_1_transform.child_frame_id = "rslidar_front";
            cloud_2_transform.child_frame_id = "rslidar_back";

            cloud_1_transform.transform.translation.x = 3.165;
            cloud_1_transform.transform.translation.y = 1.345;
            cloud_1_transform.transform.translation.z = 1.0;
            cloud_1_transform.transform.rotation.w = 0.0;
            cloud_1_transform.transform.rotation.x = 1.0;
            cloud_1_transform.transform.rotation.y = 0.0;
            cloud_1_transform.transform.rotation.z = 0.0;

            cloud_2_transform.transform.translation.x = -3.165;
            cloud_2_transform.transform.translation.y = -1.345;
            cloud_2_transform.transform.translation.z = 1.0;
            cloud_2_transform.transform.rotation.w = 0.0;
            cloud_2_transform.transform.rotation.x = 0.0;
            cloud_2_transform.transform.rotation.y = -1.0;
            cloud_2_transform.transform.rotation.z = 0.0;

            tf2::doTransform(cloud_1, cloud_1_out, cloud_1_transform);
            tf2::doTransform(cloud_2, cloud_2_out, cloud_2_transform);

            pcl::concatenatePointCloud(cloud_1_out, cloud_2_out, concatenated_cloud);
            concatenated_cloud.fields[3].name = "intensity";
            concatenated_cloud.header.frame_id = "base_link";
            

            cloud_pub_.publish(concatenated_cloud);

            concatenated_scan = *pointcloud_to_laserscan(&concatenated_cloud);
            scan_pub_.publish(concatenated_scan);
        }
    // }
}

void lidar_front(const sensor_msgs::PointCloud2ConstPtr& point_msg){
    cloud_1 = *point_msg;
    c1 = true;
    //ROS_INFO("1");
    concat_with_pc();
}

void lidar_back(const sensor_msgs::PointCloud2ConstPtr& point_msg){
    cloud_2 = *point_msg;
    c2 = true;
    //ROS_INFO("2");
    //concat_with_pc();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
    scan_pub_ = n.advertise<sensor_msgs::LaserScan>("scan", 1);
    cloud_pub_ = n.advertise<sensor_msgs::PointCloud2>("concatenated_cloud", 1);

    ros::Subscriber sub_front_rslidar_points;
    ros::Subscriber sub_back_rslidar_points;
    sub_front_rslidar_points = n.subscribe("/front/rslidar_points", 1, lidar_front);
    sub_back_rslidar_points = n.subscribe("/back/rslidar_points", 1, lidar_back);

    n.getParam("min_ang", min_ang_);
    n.getParam("max_ang", max_ang_);
    n.getParam("range_min", range_min_);
    n.getParam("range_max", range_max_);
    n.getParam("frame_id", frame_id_);

    ros::spin();
}