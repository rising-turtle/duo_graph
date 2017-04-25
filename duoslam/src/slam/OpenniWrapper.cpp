#include "OpenniWrapper.h"
//Documentation see header file
#include "pcl/ros/conversions.h"
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
//#include "pcl/common/transform.h"
#include "pcl_ros/transforms.h"
#include "openni_listener.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <cv.h>
//#include <ctime>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Core>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "NodeWrapper.h"
#include "misc.h"
//#include <image_geometry/pinhole_camera_model.h>
//#include "pcl/ros/for_each_type.h"

//For rosbag reading
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include "parameter_server.h"
#include "scoped_timer.h"
//for comparison with ground truth from mocap and movable cameras on robots
#include <tf/transform_listener.h>
#include "GraphWrapper.h"
#include "DuoGraph.h"

typedef message_filters::Subscriber<sensor_msgs::Image> image_sub_type;      
typedef message_filters::Subscriber<sensor_msgs::CameraInfo> cinfo_sub_type;      
typedef message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_type;      
typedef message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_type;      

// COpenniWrapper::COpenniWrapper(GraphManager* g_mgr):
//    OpenNIListener(g_mgr)
COpenniWrapper::COpenniWrapper(CDuoGraph* g_mgr) : 
    OpenNIListener(g_mgr->getFrontGraph()),
    m_duo_mgr(g_mgr)
{
    ParameterServer* ps = ParameterServer::instance();
    int q = ps->get<int>("subscriber_queue_size");
    std::string bagfile_name = ps->get<std::string>("bagfile_name");
    std::string visua_tpc = ps->get<std::string>("topic_image_mono");
    std::string depth_tpc = ps->get<std::string>("topic_image_depth");
    std::string cinfo_tpc = ps->get<std::string>("camera_info_topic");
    ros::NodeHandle nh;
    tflistener_ = new tf::TransformListener(nh);

    //my publishers
    slamLost_pub = nh.advertise<std_msgs::String>("SlamLost", 1000);


     if(no_cloud_sync_ != NULL)
    {
	if(bagfile_name.empty())
	{
	    delete no_cloud_sync_;
	    delete visua_sub_;
	    delete depth_sub_;
	    delete cinfo_sub_;
	    ROS_INFO("no_cloud_sync succeed, override it!");

	    visua_sub_ = new image_sub_type(nh, visua_tpc, q);
	    depth_sub_ = new image_sub_type(nh, depth_tpc, q);
	    cinfo_sub_ = new cinfo_sub_type(nh, cinfo_tpc, q);
	    no_cloud_sync_ = new message_filters::Synchronizer<NoCloudSyncPolicy>(NoCloudSyncPolicy(q),  *visua_sub_, *depth_sub_, *cinfo_sub_);
	}
	else
	{ 
	    delete no_cloud_sync_;
	    delete rgb_img_sub_;
	    delete depth_img_sub_;
	    delete cam_info_sub_;
	    ROS_INFO("no_cloud_sync succeed, override it!");

	    depth_img_sub_ = new BagSubscriber<sensor_msgs::Image>();
	    rgb_img_sub_ = new BagSubscriber<sensor_msgs::Image>();
	    cam_info_sub_ = new BagSubscriber<sensor_msgs::CameraInfo>();
	    no_cloud_sync_ = new message_filters::Synchronizer<NoCloudSyncPolicy>(NoCloudSyncPolicy(q),  *rgb_img_sub_, *depth_img_sub_, *cam_info_sub_);
	}
	no_cloud_sync_->registerCallback(boost::bind(&COpenniWrapper::noCloudCallback, this, _1, _2, _3));
    }
}

COpenniWrapper::~COpenniWrapper(){}

void COpenniWrapper::noCloudCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,
                                      const sensor_msgs::ImageConstPtr& depth_img_msg,
                                      const sensor_msgs::CameraInfoConstPtr& cam_info_msg) 
{
  ScopedTimer s(__FUNCTION__);
  ROS_WARN_ONCE_NAMED("eval", "First RGBD-Data Received");
  ROS_DEBUG("Received data from kinect");
  ParameterServer* ps = ParameterServer::instance();

  if(++data_id_ < ps->get<int>("skip_first_n_frames") 
     || data_id_ % ps->get<int>("data_skip_step") != 0)
  { 
  // If only a subset of frames are used, skip computations but visualize if gui is running
    ROS_INFO_THROTTLE(1, "Skipping Frame %i because of data_skip_step setting (this msg is only shown once a sec)", data_id_);
    if(ps->get<bool>("use_gui")){//Show the image, even if not using it
      //sensor_msgs::CvBridge bridge;
      cv::Mat depth_float_img = cv_bridge::toCvCopy(depth_img_msg)->image;
      //const cv::Mat& depth_float_img_big = cv_bridge::toCvShare(depth_img_msg)->image;
      cv::Mat visual_img =  cv_bridge::toCvCopy(visual_img_msg)->image;
      //const cv::Mat& visual_img_big =  cv_bridge::toCvShare(visual_img_msg)->image;
      //cv::Mat visual_img, depth_float_img;
      //cv::resize(visual_img_big, visual_img, cv::Size(), 0.25, 0.25);
      //cv::resize(depth_float_img_big, depth_float_img, cv::Size(), 0.25, 0.25);
      if(visual_img.rows != depth_float_img.rows || 
         visual_img.cols != depth_float_img.cols){
        ROS_ERROR("depth and visual image differ in size! Ignoring Data");
        return;
      }
      depthToCV8UC1(depth_float_img, depth_mono8_img_); //float can't be visualized or used as mask in float format TODO: reprogram keypoint detector to use float values with nan to mask
      image_encoding_ = visual_img_msg->encoding;
      Q_EMIT newVisualImage(cvMat2QImage(visual_img, 0)); //visual_idx=0
      Q_EMIT newDepthImage (cvMat2QImage(depth_mono8_img_,1));//overwrites last cvMat2QImage
    }
    return;
  }
//Convert images to OpenCV format
  //sensor_msgs::CvBridge bridge;
  //cv::Mat depth_float_img = bridge.imgMsgToCv(depth_img_msg);
  //cv::Mat visual_img =  bridge.imgMsgToCv(visual_img_msg);
  cv::Mat depth_float_img = cv_bridge::toCvCopy(depth_img_msg)->image;
  //const cv::Mat& depth_float_img_big = cv_bridge::toCvShare(depth_img_msg)->image;
  cv::Mat visual_img;
  if(image_encoding_ == "bayer_grbg8"){
    cv_bridge::toCvShare(visual_img_msg);
    ROS_INFO("Converting from Bayer to RGB");
    cv::cvtColor(cv_bridge::toCvCopy(visual_img_msg)->image, visual_img, CV_BayerGR2RGB, 3);
  } else{
    ROS_DEBUG_STREAM("Encoding: " << visual_img_msg->encoding);
    visual_img =  cv_bridge::toCvCopy(visual_img_msg)->image;
  }
  //const cv::Mat& visual_img_big =  cv_bridge::toCvShare(visual_img_msg)->image;
  //cv::Size newsize(320, 240);
  //cv::Mat visual_img(newsize, visual_img_big.type()), depth_float_img(newsize, depth_float_img_big.type());
  //cv::resize(visual_img_big, visual_img, newsize);
  //cv::resize(depth_float_img_big, depth_float_img, newsize);
  if(visual_img.rows != depth_float_img.rows || 
     visual_img.cols != depth_float_img.cols){
     ROS_ERROR("depth and visual image differ in size! Ignoring Data");
     return;
  }
  image_encoding_ = visual_img_msg->encoding;

  depthToCV8UC1(depth_float_img, depth_mono8_img_); //float can't be visualized or used as mask in float format TODO: reprogram keypoint detector to use float values with nan to mask

  if(asyncFrameDrop(depth_img_msg->header.stamp, visual_img_msg->header.stamp)) 
    return;

  if (bagfile_mutex.tryLock() && save_bag_file){
     // todo: make the names dynamic
     bag.write("/camera/rgb/image_mono", ros::Time::now(), visual_img_msg);
     bag.write("/camera/depth/image", ros::Time::now(), depth_img_msg);
     ROS_INFO_STREAM("Wrote to bagfile " << bag.getFileName());
     bagfile_mutex.unlock();
  }

  if(ps->get<bool>("use_gui")){
    Q_EMIT newVisualImage(cvMat2QImage(visual_img, 0)); //visual_idx=0
    Q_EMIT newDepthImage (cvMat2QImage(depth_mono8_img_,1));//overwrites last cvMat2QImage
  }
  if(pause_ && !getOneFrame_) return;

  noCloudCameraCallback(visual_img, depth_float_img, depth_mono8_img_, depth_img_msg->header, cam_info_msg);
}
void COpenniWrapper::noCloudCameraCallback(cv::Mat visual_img, 
                                           cv::Mat depth, 
                                           cv::Mat depth_mono8_img,
                                           std_msgs::Header depth_header,
                                           const sensor_msgs::CameraInfoConstPtr& cam_info)
{
  if(getOneFrame_) { //if getOneFrame_ is set, unset it and skip check for  pause
      getOneFrame_ = false;
  } else if(pause_ && !save_bag_file) { //Visualization and nothing else
    return; 
  }
  ScopedTimer s(__FUNCTION__);
  //######### Main Work: create new node ##############################################################
  Q_EMIT setGUIStatus("Computing Keypoints and Features");
  // ROS_INFO("Yes!!!!!!!!! Succeed to override the callback function!!!!!");
  // static int ncout = 0;
  // mean shift filter depth
  // cv::Mat filtered_depth;
  // cvSmooth(depth, filtered_depth, CV_BLUR, 3, 3);

  // Node* node_ptr = new Node(visual_img, depth, depth_mono8_img, cam_info, depth_header, detector_, extractor_);
  Node* node_ptr = new CNodeWrapper(visual_img, depth, depth_mono8_img, cam_info, depth_header, detector_, extractor_);

  // ROS_INFO("After Construct %d node!", ncout);
  retrieveTransformations(depth_header, node_ptr);//Retrieve the transform between the lens and the base-link at capturing time;

  callProcessing(visual_img, node_ptr);
}

void COpenniWrapper::callProcessing(cv::Mat visual_img, Node* node_ptr)
{
    if(!future_.isFinished())
    {
        ROS_INFO("wait for graph adding node!");
        future_.waitForFinished();
    }
    visualization_img_ = visual_img; //No copy
    visualization_depth_mono8_img_ = depth_mono8_img_;//No copy
    if(ParameterServer::instance()->get<bool>("use_gui"))
    {
        //visual_img points to the data received in the callback - which might be deallocated after the callback returns. 
        //This will happen before visualization if processNode is running in background, therefore the data needs to be cloned
        visualization_img_ = visual_img.clone();
        visualization_depth_mono8_img_ = depth_mono8_img_.clone();
    }     
    future_ = QtConcurrent::run(this, &COpenniWrapper::processNode, node_ptr);      
}

void COpenniWrapper::processNode(Node* new_node)
{
    Q_EMIT setGUIStatus("Adding Node to Graph");
    bool has_been_added = m_duo_mgr->addNode(new_node);

    if(m_duo_mgr->m_bhas_switched)
    {
        graph_mgr_ = m_duo_mgr->getFrontGraph();
        m_duo_mgr->m_bhas_switched = false;
    }
    //######### Visualization code  #############################################
    if(ParameterServer::instance()->get<bool>("use_gui"))
    {
    	std_msgs::String msg;
    	std::stringstream slamLost;

    	if(has_been_added)
    	{
    		//save pcd and traj of every frames for linyuan
    		/*
    		static int node_id = 0;
    		CGraphWrapper* pg = m_duo_mgr->getFrontGraph();
            g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(pg->getOptimizer()->vertex(new_node->vertex_id_));
            if(!v)
            {
                ROS_ERROR("Nullpointer in graph at position!");
            }
            else
            {
            	node_id++;
            	//save organized pcd and its pose
        		char posename[255];
        		sprintf(posename, "./submaps/pose_%.06d.txt",node_id);
        		ofstream ofs_pose(posename);

        		char pcdname[255];
        		sprintf(pcdname, "./submaps/pcd_%.06d.pcd",node_id);
        		ofstream ofs_pcd(pcdname);

        		QMatrix4x4 qmat = g2o2QMatrix(v->estimateAsSE3Quat());
        		ofs_pose<<qmat(0,0)<<" "<<qmat(0,1)<<" "<<qmat(0,2)<<" "<<qmat(0,3)<<" ";
        		ofs_pose<<qmat(1,0)<<" "<<qmat(1,1)<<" "<<qmat(1,2)<<" "<<qmat(1,3)<<" ";
        		ofs_pose<<qmat(2,0)<<" "<<qmat(2,1)<<" "<<qmat(2,2)<<" "<<qmat(2,3)<<" ";
        		ofs_pose<<qmat(3,0)<<" "<<qmat(3,1)<<" "<<qmat(3,2)<<" "<<qmat(3,3)<<" ";
            	ofs_pose.close();
            	pcl::io::savePCDFile(pcdname, *(new_node->pc_col), false);
            	ROS_ERROR("Saved PCD File %s and %s", posename, pcdname);
            }
            */



    		/*
    		ROS_ERROR("ADDED");
    		  ofstream ofs("featuresForSH.txt");
    		  for(int i=0;i<new_node->feature_locations_2d_.size();i++)
    		  {
    			  ofs<<new_node->feature_locations_2d_[i].pt.x<<" "<<new_node->feature_locations_2d_[i].pt.y<<" ";
    			  ofs<<new_node->feature_locations_3d_[i].x()<<" "<<new_node->feature_locations_3d_[i].y()<<" "<<new_node->feature_locations_3d_[i].z()<<" ";
    			  for(int j=0;j<new_node->feature_descriptors_.cols;j++)
    			  {
    				  ofs<<new_node->feature_descriptors_.at<float>(i,j)<<" ";
    			  }
    			  ofs<<endl;
    		  }
    		  */


    		//publish slamlost info, if working it is "No"
    		slamLost << "No";
    		msg.data = slamLost.str();

    		if(ParameterServer::instance()->get<bool>("visualize_mono_depth_overlay"))
    		{
    			cv::Mat feature_img = cv::Mat::zeros( visualization_img_.rows, visualization_img_.cols, CV_8UC1);
    			graph_mgr_->drawFeatureFlow(feature_img);
    			Q_EMIT newFeatureFlowImage(cvMat2QImage(visualization_img_,visualization_depth_mono8_img_, feature_img, 2)); //show registration
    		} else
    		{
    			graph_mgr_->drawFeatureFlow(visualization_img_, cv::Scalar(0,0,255), cv::Scalar(0,128,0) );
                Q_EMIT newFeatureFlowImage(cvMat2QImage(visualization_img_, 2)); //show registration
            }
        } else 
        {
        	//publish lost info
    		slamLost << "Yes";
    		msg.data = slamLost.str();

            if(ParameterServer::instance()->get<bool>("visualize_mono_depth_overlay")){
                cv::Mat feature_img = cv::Mat( visualization_img_.rows, visualization_img_.cols, CV_8UC1); 
                cv::drawKeypoints(feature_img, new_node->feature_locations_2d_, feature_img, cv::Scalar(155), 5);
                Q_EMIT newFeatureFlowImage(cvMat2QImage(visualization_img_,visualization_depth_mono8_img_, feature_img, 2)); //show registration
            } else 
            {
                cv::drawKeypoints(visualization_img_, new_node->feature_locations_2d_, visualization_img_, cv::Scalar(0, 100,0), 5);
                Q_EMIT newFeatureFlowImage(cvMat2QImage(visualization_img_, 2)); //show registration
            }
        }
    	slamLost_pub.publish(msg);
    }
    if(!has_been_added) delete new_node;
    
    //######### Submap Swap code #############################################
    // static CGraphWrapper* pg = dynamic_cast<CGraphWrapper*>(graph_mgr_);
    // if(pg != 0) pg->submapSwap();
}



