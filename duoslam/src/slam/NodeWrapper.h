#ifndef NODE_WRAPPER_H
#define NODE_WRAPPER_H
#include "parameter_server.h"
#include "paramSrvMi.h"
#include "node.h"
#include "matching_result.h"

class CNodeWrapper : public Node
{
public:
    	CNodeWrapper(const cv::Mat& visual,
			 const cv::Mat& depth,
			 const cv::Mat& detection_mask,
       const sensor_msgs::CameraInfoConstPtr& cam_info, 
       std_msgs::Header depth_header,
			 cv::Ptr<cv::FeatureDetector> detector,
			 cv::Ptr<cv::DescriptorExtractor> extractor);
        CNodeWrapper(const cv::Mat visual,
			 cv::Ptr<cv::FeatureDetector> detector,
			 cv::Ptr<cv::DescriptorExtractor> extractor,
			 pointcloud_type::Ptr point_cloud,
			 const cv::Mat detection_mask = cv::Mat());
        CNodeWrapper(); // needed for submap_node
        virtual ~CNodeWrapper();
        CNodeWrapper(const CNodeWrapper&);
        CNodeWrapper& operator=(const CNodeWrapper&);
        void Init();
               
        // virtual MatchingResult matchNodePair(const Node* );
public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif
