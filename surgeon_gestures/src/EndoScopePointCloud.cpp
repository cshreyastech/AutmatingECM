#include "surgeon_gestures/EndoScopePointCloud.h"

EndoScopePointCloud::EndoScopePointCloud(ros::NodeHandle *nh)
{
    // Create a ROS subscriber for the input point cloud
    sub_ = nh->subscribe("/ambf/env/cameras/endo_camera/DepthData", 1,
                         &EndoScopePointCloud::cloud_callback, this);

    // Create a ROS publisher for the output point cloud
    pub_ = nh->advertise<sensor_msgs::PointCloud2> ("/transformed_cloud", 1);

    rate_ = new ros::Rate(rate_hz_);

    this->transformPointCloud();

}

void EndoScopePointCloud::cloud_callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Create a container for the data.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    pcl_ros::transformPointCloud(*temp_cloud, *cloud_transformed, transform_);
    sensor_msgs::PointCloud2 cloud_publish;
    pcl::toROSMsg(*cloud_transformed,cloud_publish);
    cloud_publish.header = input->header;
    cloud_publish.header.frame_id = child_;

    pub_.publish(cloud_publish);

}


void EndoScopePointCloud::transformPointCloud() {
    tf::TransformListener listener;
    static tf2_ros::TransformBroadcaster br;

    double roll,pitch, yaw;
    roll = -0.5*M_PI;
    pitch = -0.5*M_PI;
    yaw = 0;

//    parent = "/depth_camera";
//    child = "/AMBF_camera";

//    while (ros::ok() )
//    {
    transform_.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    transform_.setRotation( tf::Quaternion(roll, pitch, yaw) );
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = parent_;
    transformStamped.child_frame_id = child_;
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    br.sendTransform(transformStamped);
//        ros::spinOnce();
//    rate_.sleep();

    this->rateSleep();
//    }
}

EndoScopePointCloud::~EndoScopePointCloud(void) {

}

int main (int argc, char** argv)
{
    // Initialize ROS
    ROS_INFO("Node started");

//    ros::init (argc, argv, "point_cloud_transform");
//    ros::NodeHandle nh;
//    ros::Rate rate(1000); // ROS Rate at 5Hz

//    // Create a ROS subscriber for the input point cloud
//    ros::Subscriber sub = nh.subscribe ("/ambf/env/cameras/cameraEndoscope/DepthData", 1, cloud_callback);

//    // Create a ROS publisher for the output point cloud
//    pub = nh.advertise<sensor_msgs::PointCloud2> ("/transformed_cloud", 1);

//    tf::TransformListener listener;
//    static tf2_ros::TransformBroadcaster br;

      ros::init(argc, argv, "point_cloud_transform");
      ros::NodeHandle nh;
      EndoScopePointCloud ecmEndoScopePointCloud(&nh);


      ros::spin();
//    while (ros::ok()) {
////        ecmEndoScopePointCloud.navigateBB8();
//        ecmEndoScopePointCloud.rateSleep();
//        ros::spinOnce();
//    }

    return 0;
}
