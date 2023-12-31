#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
using namespace sensor_msgs;
using namespace message_filters;
// cv_bridge::CvImageConstPtr cv_bottom_ptr, cv_front_left_ptr, cv_front_right_ptr;
int fx=320;
int fy=320;
int cx=320;
int cy=240;
int baseline=0.095;
ros::Publisher pcl_pub;
cv::Ptr<cv::StereoBM> left_matcher = cv::StereoBM::create();
cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);
// std::unique_ptr<image_transport::ImageTransport> it;

void imageCallback(const sensor_msgs::ImageConstPtr& msg1, const sensor_msgs::ImageConstPtr& msg2)
{
    cv_bridge::CvImagePtr cv_ptr1;
    cv_bridge::CvImagePtr cv_ptr2;

    try
    {
        cv_ptr1 = cv_bridge::toCvCopy(msg1, sensor_msgs::image_encodings::TYPE_8UC3);
        cv_ptr2 = cv_bridge::toCvCopy(msg2, sensor_msgs::image_encodings::TYPE_8UC3);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat left_img = cv_ptr1->image;
    cv::Mat right_img = cv_ptr2->image;
    cv::Mat left_disp, right_disp, filtered_disp;
    
    left_matcher->compute(left_img, right_img, left_disp);

    // Calculate filtered disparity map.
    wls_filter->filter(left_disp, left_img, filtered_disp);

    // Create point cloud.
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (int i = 0; i < filtered_disp.rows; ++i)
    {
        for (int j = 0; j < filtered_disp.cols; ++j)
        {
            float disparity = filtered_disp.at<float>(i, j);
            if (disparity > 0)
            {
                pcl::PointXYZ point;
                point.x = (j - cx) * baseline / disparity;
                point.y = (i - cy) * baseline / disparity;
                point.z = fx * baseline / disparity;
                cloud.points.push_back(point);
            }
        }
    }

    // Publish the point cloud.
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "";
    pcl_pub.publish(output);
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "img_pcl");
    ros::NodeHandle nh;
    // it = std::make_unique<image_transport::ImageTransport>(nh); 
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/airsim_piontcloud", 1);
    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/airsim_node/drone_1/front_left/Scene", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/airsim_node/drone_1/front_right/Scene", 1);
    TimeSynchronizer<sensor_msgs::Image,sensor_msgs::Image> sync(left_sub, right_sub, 10);
    sync.registerCallback(boost::bind(&imageCallback, _1, _2));
    ros::spin();
    return 0;
}
