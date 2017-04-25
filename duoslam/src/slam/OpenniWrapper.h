#ifndef OPENNI_WRAPPER_H
#define OPENNI_WRAPPER_H
#include "openni_listener.h"

class CDuoGraph;

class COpenniWrapper : public OpenNIListener 
{
    Q_OBJECT
public:
    // COpenniWrapper(GraphManager* g_mgr);
    COpenniWrapper(CDuoGraph* );
    virtual ~COpenniWrapper();
Q_SIGNALS:
    // void sendNode2Graph(void*);
public:
    //! For this callback the point cloud is not required. 
    void noCloudCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,
                          const sensor_msgs::ImageConstPtr& depth_img_msg,
                          const sensor_msgs::CameraInfoConstPtr& cam_info_msg) ;

    //! as cameraCallback, but create Node without cloud
    void noCloudCameraCallback(cv::Mat visual_img, 
                               cv::Mat depth, 
                               cv::Mat depth_mono8_img,
                               std_msgs::Header depth_header,
                               const sensor_msgs::CameraInfoConstPtr& cam_info);

    //!Call processNode either regularly or as background thread
    void callProcessing(cv::Mat gray_img, Node* node_ptr);
    void processNode(Node* node_ptr);
    CDuoGraph * m_duo_mgr;

    ros::Publisher slamLost_pub;

};


#endif
