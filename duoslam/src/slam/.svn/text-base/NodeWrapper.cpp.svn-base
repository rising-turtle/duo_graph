#include "NodeWrapper.h"
#include "pcl/registration/transformation_estimation.h"
#include <cmath>
// #include "scoped_timer.h"
#include <Eigen/Geometry>
#include <pcl/common/transformation_from_correspondences.h>
#include <fstream>
#include "misc.h"
#include <pcl/filters/voxel_grid.h>
#include <opencv/highgui.h>
#include "pcl/registration/icp.h"
#include <QMutex>

QMutex Node::gicp_mutex;
QMutex Node::siftgpu_mutex;

void CNodeWrapper::Init()
{
    bool resetPC = false;
    bool resetGICP = false;
    ParameterServer* ps = ParameterServer::instance();
    /*
#ifndef USE_PCL_ICP
    resetPC = true;
#else
    if(!ps->get<bool>("use_icp"))
        resetPC = true;
#endif    

#ifndef USE_ICP_CODE
    resetGICP = true;
#else
    if(!(!ps->get<bool>("store_pointclouds") && ps->get<bool>("use_icp")))
        resetGICP = true;
#endif

    if(resetPC)
    {
        filterCloud(*pc_col, *filtered_pc_col, ps->get<int>("gicp_max_cloud_size")); 
    }
    
    if(resetGICP)
    {
        gicp_initialized = false;
        gicp_point_set_ = NULL;
        {//if clearing out point clouds, the icp structure needs to be built before
            gicp_mutex.lock();
            gicp_point_set_ = this->getGICPStructure();
            gicp_mutex.unlock();
        } 
    }
    ROS_INFO("before set gicp debug!");
     gicp_mutex.lock();
        dgc::gicp::GICPPointSet* gicp_point_set = this->getGICPStructure();
        gicp_point_set->SetDebug(false);
     gicp_mutex.unlock();
     */
}

CNodeWrapper::CNodeWrapper(const cv::Mat& visual, 
           const cv::Mat& depth,
           const cv::Mat& detection_mask,
           const sensor_msgs::CameraInfoConstPtr& cam_info, 
           std_msgs::Header depth_header,
           cv::Ptr<cv::FeatureDetector> detector,
           cv::Ptr<cv::DescriptorExtractor> extractor)
    : Node(visual, depth, detection_mask, cam_info, depth_header, detector, extractor)
{
    Init();
}

CNodeWrapper::CNodeWrapper(const cv::Mat visual,
           cv::Ptr<cv::FeatureDetector> detector,
           cv::Ptr<cv::DescriptorExtractor> extractor,
           pointcloud_type::Ptr point_cloud,
           const cv::Mat detection_mask)
    : Node(visual, detector, extractor, point_cloud, detection_mask)
{
    Init();
}

CNodeWrapper::CNodeWrapper(){}

CNodeWrapper::~CNodeWrapper(){}

CNodeWrapper::CNodeWrapper(const CNodeWrapper& rhs)
{
    // 1 ids
    id_ = rhs.id_;
    seq_id_ = rhs.seq_id_;
    vertex_id_ = rhs.vertex_id_;
    matchable_ = rhs.matchable_;
    initial_node_matches_ = rhs.initial_node_matches_;

    // 2 point cloud
    pc_col = rhs.pc_col->makeShared();
    
    // 3 flann, this will be created when matched
    flannIndex = NULL;
    
    // 4 transform
    valid_tf_estimate_ = rhs.valid_tf_estimate_;
    base2points_ = rhs.base2points_;
    ground_truth_transform_ = rhs.ground_truth_transform_;
    odom_transform_ = rhs.odom_transform_;
    
    // 5 features in 2D
    feature_locations_2d_.resize(rhs.feature_locations_2d_.size());
    for(int i=0;i<feature_locations_2d_.size();i++)
    {
        feature_locations_2d_[i] = rhs.feature_locations_2d_[i];
    }
    unsigned int N = rhs.feature_depth_stats_.size();
    for(int i=0; i<N; i++)
    {
        const pair<float, float>& item = rhs.feature_depth_stats_[i];
        feature_depth_stats_.push_back(make_pair(item.first, item.second));
    }
    feature_matching_stats_.insert(feature_matching_stats_.begin(),
        rhs.feature_matching_stats_.begin(),rhs.feature_matching_stats_.end());
    
    // 6 feature in 3D
    feature_locations_3d_.insert(feature_locations_3d_.begin(), 
        rhs.feature_locations_3d_.begin(), rhs.feature_locations_3d_.end());
    feature_descriptors_ = rhs.feature_descriptors_.clone();

#ifdef  DO_FEATURE_OPTIMIZATION
    // 7 landmark 
    // std::map<int, int> kpt_to_landmark;
    kpt_to_landmark.clear();
    map<int, int>::const_iterator it_rhs = rhs.kpt_to_landmark.begin();
    while(it_rhs!=rhs.kpt_to_landmark.end())
    {
        kpt_to_landmark[it_rhs->first] = it_rhs->second;
        ++it_rhs;
    }
#endif
    
// TODO:
#ifdef USE_PCL_ICP
     // filtered_pc_col(new pointcloud_type()),
#endif
#ifdef USE_ICP_CODE
#endif
#ifdef USE_SIFT_GPU
#endif
}

CNodeWrapper& CNodeWrapper::operator=(const CNodeWrapper& rhs)
{   
    if(this == &rhs)
    {
        return (*this);
    }
        // 1 ids
    id_ = rhs.id_;
    seq_id_ = rhs.seq_id_;
    vertex_id_ = rhs.vertex_id_;
    matchable_ = rhs.matchable_;
    initial_node_matches_ = rhs.initial_node_matches_;

    // 2 point cloud
    pc_col = rhs.pc_col->makeShared();
    
    // 3 flann, this will be created when matched
    flannIndex = NULL;
    
    // 4 transform
    valid_tf_estimate_ = rhs.valid_tf_estimate_;
    base2points_ = rhs.base2points_;
    ground_truth_transform_ = rhs.ground_truth_transform_;
    odom_transform_ = rhs.odom_transform_;
    
    // 5 features in 2D
    feature_locations_2d_.resize(rhs.feature_locations_2d_.size());
    for(int i=0;i<feature_locations_2d_.size();i++)
    {
        feature_locations_2d_[i] = rhs.feature_locations_2d_[i];
    }
    unsigned int N = rhs.feature_depth_stats_.size();
    for(int i=0; i<N; i++)
    {
        const pair<float, float>& item = rhs.feature_depth_stats_[i];
        feature_depth_stats_.push_back(make_pair(item.first, item.second));
    }
    feature_matching_stats_.insert(feature_matching_stats_.begin(),
        rhs.feature_matching_stats_.begin(),rhs.feature_matching_stats_.end());
    
    // 6 feature in 3D
    feature_locations_3d_.insert(feature_locations_3d_.begin(), 
        rhs.feature_locations_3d_.begin(), rhs.feature_locations_3d_.end());
    feature_descriptors_ = rhs.feature_descriptors_.clone();

#ifdef  DO_FEATURE_OPTIMIZATION
    // 7 landmark 
    // std::map<int, int> kpt_to_landmark;
    kpt_to_landmark.clear();
    map<int, int>::const_iterator it_rhs = rhs.kpt_to_landmark.begin();
    while(it_rhs!=rhs.kpt_to_landmark.end())
    {
        kpt_to_landmark[it_rhs->first] = it_rhs->second;
        ++it_rhs;
    }
#endif
    
// TODO:
#ifdef USE_PCL_ICP
     // filtered_pc_col(new pointcloud_type()),
#endif
#ifdef USE_ICP_CODE
#endif
#ifdef USE_SIFT_GPU
#endif
    return (*this);
}

/*
MatchingResult CNodeWrapper::matchNodePair(const Node* older_node)
{
  MatchingResult mr;
  ///First check if this node has the information required
  if(older_node->pc_col->size() == 0 || older_node->feature_locations_2d_.size() == 0){
    // ROS_WARN("Tried to match against a cleared node (%d). Skipping.", older_node->id_); 
    return mr;
  }
  ParameterServer* ps = ParameterServer::instance();
  if(ps->get<int>("max_connections") > 0 && initial_node_matches_ > ps->get<int>("max_connections")) {
    return mr; //enough is enough
  }

  try{
    ///FEATURE MATCHING+RANSAC
    bool found_transformation = false;

    this->featureMatching(older_node, &mr.all_matches); 
    double ransac_quality = 0;
    if (mr.all_matches.size() < (unsigned int) ps->get<int>("min_matches")){
        // ROS_INFO("Too few inliers between %i and %i for RANSAC method. Only %i correspondences to begin with.",
         //        older_node->id_,this->id_,(int)mr.all_matches.size());
    } 
    else {//All good for feature based transformation estimation
        if(getRelativeTransformationTo(older_node,&mr.all_matches, mr.ransac_trafo, mr.rmse, mr.inlier_matches))
        {
          pairwiseObservationLikelihood(this, older_node, mr);
          bool valid_tf = observation_criterion_met(mr.inlier_points, mr.outlier_points, mr.occluded_points + mr.inlier_points + mr.outlier_points, ransac_quality);
          if(valid_tf){
            edgeFromMatchingResult(this, older_node, mr.ransac_trafo, mr);
            // printNNRatioInfo("valid", mr.inlier_matches);
            found_transformation = true;
          }
        } 
        else {//Informational output only
          // printNNRatioInfo("invalid", mr.all_matches);
          ROS_INFO("RANSAC match fails!");
        }
        if(!found_transformation) mr.inlier_matches.clear();
    } 
    
    // ros::Time before_icp = ros::Time::now();
    ///ICP - This sets the icp transformation in "mr", if the icp alignment is better than the ransac_quality
    // found_transformation = edge_from_icp_alignment(found_transformation, this, older_node, mr, ransac_quality) || found_transformation;
    // ros::Time after_icp = ros::Time::now();
    // long icp_duration = abs(static_cast<long>(after_icp.nsec) - static_cast<long>(before_icp.nsec));
    // long icp_duration = ms_rostimeDiff(before_icp, after_icp);
    // if(icp_duration > 0)
       //  ROS_INFO("!!~ ICP cost time: %ld ",icp_duration);

    if(found_transformation) {
        // ROS_INFO("Returning Valid Edge");
        ++initial_node_matches_; //trafo is accepted
    } else {
        mr.edge.id1 = mr.edge.id2 = -1;
    }
  }
  catch (std::exception e){//Catch exceptions: Unexpected problems shouldn't crash the application
    ROS_ERROR("Caught Exception in comparison of Nodes %i and %i: %s", this->id_, older_node->id_, e.what());
  }
  return mr;
}*/




