#ifndef NODE_HOLDER_H
#define NODE_HOLDER_H

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/features2d/features2d.hpp>
#include <pcl/visualization/cloud_viewer.h>

class CNodeWrapper; 
class MatchingResult; 

class CNodeHolder
{
public:
    CNodeHolder(cv::Mat, cv::Mat ,\
            const sensor_msgs::CameraInfoConstPtr& caminfo, const std_msgs::Header& );
    ~CNodeHolder();
    void matchNodePair(CNodeHolder*, MatchingResult& mr);
    void showPC(pcl::visualization::CloudViewer&);

    void swap(CNodeHolder*); 
    CNodeWrapper* getNode(); //{return m_pNode;}
    void setNode(CNodeWrapper* t){m_pNode = t;}

private:
    cv::Ptr<cv::FeatureDetector> m_detector; 
    cv::Ptr<cv::DescriptorExtractor> m_extractor;
    CNodeWrapper* m_pNode;
};


#endif
