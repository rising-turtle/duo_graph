#ifndef RECEPUB_H
#define RECEPUB_H

#include "ros/ros.h"
//#include <pcl_tf/transforms.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/core/core.hpp>

//The policy merges kinect messages with approximately equal timestamp into one callback 
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                       sensor_msgs::Image,
                                                       sensor_msgs::CameraInfo> NoCloudSyncPolicy;

class CRecePub
{
public:
    CRecePub();
    ~CRecePub();

    //! For this callback the point cloud is not required. 
    void noCloudCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,
            const sensor_msgs::ImageConstPtr& depth_img_msg,
            const sensor_msgs::CameraInfoConstPtr& cam_info_msg) ;
    //! as cameraCallback, but create Node without cloud
    /*void noCloudCameraCallback(cv::Mat visual_img,
            cv::Mat depth,
            cv::Mat depth_mono8_img,
            std_msgs::Header depth_header,
            const sensor_msgs::CameraInfoConstPtr& cam_info);*/
    message_filters::Synchronizer<NoCloudSyncPolicy>* no_cloud_sync_;
    message_filters::Subscriber<sensor_msgs::Image> *visua_sub_;
    message_filters::Subscriber<sensor_msgs::Image> *depth_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> *cinfo_sub_;

    cv::Mat depth_mono8_img_;
    ///The depth mono img is stored here for visualization purposes
    cv::Mat visualization_depth_mono8_img_;
    ///The visual img is stored here for visualization purposes here
    cv::Mat visualization_img_;

};


#endif
