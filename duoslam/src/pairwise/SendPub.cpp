#include <stdlib.h>
#include <iostream>
#include <string>
#include <unistd.h>

//opencv
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <opencv/cvwimage.h>

//ros
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/image_encodings.h>

using namespace std;
using namespace cv;

bool require_frame = true;

void slamLostCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("SLAM LOST: [%s]", msg->data.c_str());
    require_frame = true;
}


int main( int argc, char** argv )
{
    // ros init
    ros::init(argc, argv, "sendImg");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_img = it.advertise("camera/rgb/image_rect_color", 1);
    image_transport::Publisher pub_depth = it.advertise("camera/depth_registered/image_rect", 1);

    // get current CameraInfo data
    camera_info_manager::CameraInfoManager info_mgr(nh, "rgb");    
    sensor_msgs::CameraInfoPtr 	info(new sensor_msgs::CameraInfo(info_mgr.getCameraInfo()));
    ros::Publisher info_pub = nh.advertise<sensor_msgs::CameraInfo>("camera/rgb/camera_info", 1);

    //get lost info
    ros::Subscriber slamLostSub = nh.subscribe("SlamLost", 1000, slamLostCallback);


    unsigned int pair_id = 0;
    std::string frame = "camera";
    int img_h = 480;
    int img_w = 640;
    char key = 0;

    //read image
    int max_num_img = 80000;
    char rgbName[255];
    char depthName[255];
    int i =1 ;
    bool first = true;
    //for(int i=0;i<max_num_img && nh.ok() && (key!=27);i++)
    while(ros::ok())
    {
	if(require_frame)
	{
	    i++;
	    cout<<"img: "<<i<<endl;
	    sprintf(rgbName, "/home/davidz/Data/b2_0926_workspace/image_%.6d.png", i);
	    sprintf(depthName, "/home/davidz/Data/b2_0926_workspace/depth_%.6d.png", i);
		Mat rgb_img = imread(rgbName, -1);
	    Mat depth_img = imread(depthName, -1);
	    //	cvtColor(rgb_img,rgb_img,CV_RGB2BGR); // if the image is in rgb, have to convert it to bgr


	    ros::Time capture_time = ros::Time::now();
	    sensor_msgs::ImagePtr img_msg= boost::make_shared<sensor_msgs::Image>();

	    img_msg->height = img_h;
	    img_msg->width = img_w;
	    img_msg->step = 3 * img_w;
	    img_msg->encoding = sensor_msgs::image_encodings::BGR8;
	    img_msg->header.stamp = capture_time;
	    img_msg->header.seq = pair_id;
	    img_msg->header.frame_id = frame;
	    img_msg->data.resize(img_msg->step * img_msg->height);
	    memcpy(&img_msg->data[0], rgb_img.data, img_h*img_w*3);


	    //publish image_info
	    /* Throw out any CamInfo that's not calibrated to this camera mode */
	    if (info->K[0] != 0.0 &&
		    (img_msg->width != info->width
		     || img_msg->height != info->height)) {
		info.reset(new sensor_msgs::CameraInfo());
	    }

	    /* If we don't have a calibration, set the image dimensions */
	    if (info->K[0] == 0.0) {
		info->width = img_msg->width;
		info->height = img_msg->height;
	    }

	    info->header.stamp = img_msg->header.stamp;
	    info->header.frame_id = frame;

	    //publish depth
	    sensor_msgs::ImagePtr depth_msg = boost::make_shared<sensor_msgs::Image>();
	    depth_msg->header.stamp = capture_time;
	    depth_msg->header.seq = pair_id;
	    depth_msg->header.frame_id = "depth";
	    depth_msg->encoding        = sensor_msgs::image_encodings::TYPE_16UC1;
	    depth_msg->height          = img_h;
	    depth_msg->width           = img_w;
	    depth_msg->step            = img_w * sizeof(short);
	    depth_msg->data.resize(depth_msg->step * depth_msg->height);

	    memcpy(&depth_msg->data[0], depth_img.data, img_h*img_w*2);


	    if(first)
	    {
		imshow("Display depth",rgb_img);
		imshow("Display image",depth_img);
		key=cvWaitKey(200);

		if(key == 's')
		{
		    pub_img.publish(img_msg);
		    info_pub.publish(info);
		    pub_depth.publish(depth_msg);
		    ros::spinOnce();
		    ++pair_id;

		    first = false;
		    require_frame= false;
		}
	    }
	    else
	    {
		pub_img.publish(img_msg);
		info_pub.publish(info);
		pub_depth.publish(depth_msg);

		ros::spinOnce();
		++pair_id;
		require_frame= false;
	    }

	}
	else
	{
	    ROS_INFO("WAIT FOR SLAM FINISHED!!");
	    ros::spinOnce();
	    usleep(100000);
	}
    }

    return 0;
}
