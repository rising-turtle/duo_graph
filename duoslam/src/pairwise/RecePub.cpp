#include "RecePub.h"
#include "NodeHolder.h"
#include "NodeWrapper.h"
//#include ""
#include <pcl/io/io.h>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <pcl/io/pcd_io.h>

using namespace std;

typedef message_filters::Subscriber<sensor_msgs::Image> image_sub_type;
typedef message_filters::Subscriber<sensor_msgs::CameraInfo> cinfo_sub_type;

CRecePub::CRecePub():
visua_sub_(0), 
depth_sub_(0),
cinfo_sub_(0)
{
    unsigned int q = 1; // queue size
    string visual_tpc = "/camera/rgb/image_rect_color";
    string depth_tpc = "/camera/depth_registered/image_rect";
    string cinfo_tpc = "/camera/rgb/camera_info";
    ros::NodeHandle nh;
    visua_sub_ = new image_sub_type(nh, visual_tpc, q); 
    depth_sub_ = new image_sub_type(nh, depth_tpc, q); 
    cinfo_sub_ = new cinfo_sub_type(nh, cinfo_tpc, q); 
    no_cloud_sync_ = new message_filters::Synchronizer<NoCloudSyncPolicy>(NoCloudSyncPolicy(q),  *visua_sub_, *depth_sub_, *cinfo_sub_);
    no_cloud_sync_->registerCallback(boost::bind(&CRecePub::noCloudCallback, this, _1, _2, _3));
}

CRecePub::~CRecePub()
{

}

void CRecePub::noCloudCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,
                                const sensor_msgs::ImageConstPtr& depth_img_msg,
                                const sensor_msgs::CameraInfoConstPtr& cam_info_msg)
{
    cv::Mat depth_float_img = cv_bridge::toCvCopy(depth_img_msg)->image;
    cv::Mat visual_img =  cv_bridge::toCvCopy(visual_img_msg)->image;
    // image_encoding_ = visual_img_msg->encoding;
    // depthToCV8UC1(depth_float_img, depth_mono8_img_); //float can't be visualized or used as mask in float format TODO: reprogram keypoint detector to use float values with nan to mask
    
    CNodeHolder node(visual_img, depth_float_img, cam_info_msg, depth_img_msg->header);
    ROS_ERROR("RecePub.cpp: receive CloudCallback");
    pcl::io::savePCDFile("frame1.pcd", *(node.getNode()->pc_col));
}




