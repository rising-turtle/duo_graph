#include "NodeHolder.h"
// #include "node.h"
#include "NodeWrapper.h"
#include <sensor_msgs/CameraInfo.h>
#include "misc.h"
#include <pcl/visualization/cloud_viewer.h>
#include "matching_result.h"
#include <iostream>

//Variables
// cv::Ptr<cv::FeatureDetector> CNodeHolder::s_detector = createDetector("SURF");
// cv::Ptr<cv::DescriptorExtractor> CNodeHolder::s_extractor = createDescriptorExtractor("SURF");

CNodeHolder::CNodeHolder(cv::Mat visual_img, cv::Mat depth_float_img, 
                const sensor_msgs::CameraInfoConstPtr& caminfo, const std_msgs::Header& header):m_pNode(0) 
{
    cv::Mat depth_mono8_img_;
    // std::string image_encoding_ = visual_img.encoding;
    // cout<<"before depthToCV8UC1"<<endl;
    depthToCV8UC1(depth_float_img, depth_mono8_img_);
    m_detector = createDetector("SURF");
    m_extractor = createDescriptorExtractor("SURF");

    // cout<<"before create CNodeWrapper!"<<endl;
    m_pNode = new CNodeWrapper(visual_img, depth_float_img, depth_mono8_img_, caminfo, header, m_detector, m_extractor);
    
}

CNodeHolder::~CNodeHolder()
{
    if(m_pNode != NULL) delete m_pNode; 
    m_pNode = 0;
}

void CNodeHolder::swap(CNodeHolder* old_node)
{
    CNodeWrapper * tmp = m_pNode; 
    m_pNode = old_node->getNode();
    old_node->setNode(tmp);
}

void CNodeHolder::matchNodePair(CNodeHolder* old_node, MatchingResult& mr)
{
     mr = m_pNode->matchNodePair(old_node->getNode());      
     std::cout<<"inlier matches: "<<mr.inlier_matches.size()<<endl;
}

void CNodeHolder::showPC(pcl::visualization::CloudViewer& viewer)
{
    // pcl::visualization::CloudViewer viewer("Cloud Viewer");
    if(m_pNode != NULL)
    {
        viewer.showCloud(m_pNode->pc_col);
    }
}


CNodeWrapper* CNodeHolder::getNode(){return m_pNode;}


