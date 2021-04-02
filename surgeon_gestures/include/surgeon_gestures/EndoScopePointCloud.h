#ifndef ENDOSCOPEPOINTCLOUD_H
#define ENDOSCOPEPOINTCLOUD_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>
// PCL specific includes
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

class EndoScopePointCloud
{
public:
    EndoScopePointCloud(ros::NodeHandle *nh);
    inline void rateSleep() { rate_->sleep(); }
    ~EndoScopePointCloud(void);
private:
    void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& input);

    ros::Subscriber sub_;
    ros::Publisher pub_;

    ros::Rate *rate_;
    const float rate_hz_ = 1000;

    tf::StampedTransform transform_;
    const std::string parent_ = "/endo_camera";
    const std::string child_ = "/AMBF_camera";

    void transformPointCloud();
};

#endif // ENDOSCOPEPOINTCLOUD_H
