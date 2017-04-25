#include "SubmapGraph.h"
#include "SubmapNode.h"
#include "Submap.h"
#include "qtros.h"
#include <QObject>
#include <QApplication>
#include <iostream>
#include <sstream>
#include <string>
#include "stdlib.h"

#include "std_msgs/Float32MultiArray.h"
#include <pcl_ros/point_cloud.h>

#include "paramSrvMi.h"

using namespace std;

cv::Mat feature_loc;
cv::Mat feature_des;
pointcloud_type::Ptr submap_pcd;

bool FEATURE_LOC_READY = false;
bool FEATURE_DES_READY = false;
bool FEATURE_TF_READY = false;
bool FEATURE_CLOUD_READY = false;

//ofstream outL("featureLoc.map");
//ofstream outD("featureDes.map");

int loc_idx=0;
int des_idx=0;
int cld_idx=0;

void featureLocCallback(const std_msgs::Float32MultiArray::ConstPtr& array_msg)
{
	printf("RECEIVED feature loc: %d \n", array_msg->layout.dim[0].size);

	int fea_num = 0;
	int loc_length = 0;
	if(array_msg->layout.dim[0].label== "height")
	{
		fea_num = array_msg->layout.dim[0].size;
	}
	if(array_msg->layout.dim[1].label == "width")
	{
		loc_length = array_msg->layout.dim[1].size;
	}
	feature_loc = cv::Mat(fea_num, loc_length, CV_32FC1);


	for(int i = 0;i<fea_num; i++)
	{
		for( int j=0; j<loc_length; j++)
		{
			feature_loc.at<float>(i,j) = array_msg->data[i*loc_length+j];
		}
	}

	char name[255];
	sprintf(name, "./submaps/%d_featureLoc.map", loc_idx);
	loc_idx++;
	ofstream outL(name);
	outL<<"feature num:  "<<fea_num<<endl;
	outL<<"loc length:  "<<loc_length<<endl;
	outL<<"Data:  "<<feature_loc<<endl;
	outL.close();
	FEATURE_LOC_READY = true;

	printf("RECEIVED feature loc done! \n");
}

void featureDesCallback(const std_msgs::Float32MultiArray::ConstPtr& array_msg)
{

	printf("RECEIVED feature des: %d \n", array_msg->layout.dim[0].size);


	int fea_num = 0;
	int des_length = 0;
	if(array_msg->layout.dim[0].label == "height")
	{
		fea_num = array_msg->layout.dim[0].size;
	}
	if(array_msg->layout.dim[1].label == "width")
	{
		des_length = array_msg->layout.dim[1].size;
	}
	feature_des = cv::Mat(fea_num, des_length, CV_32FC1);

	cout<<"feature num:  "<<fea_num<<endl;
	cout<<"des length:  "<<des_length<<endl;

	for(int i = 0;i<fea_num; i++)
	{
		for( int j=0; j<des_length; j++)
		{
			feature_des.at<float>(i,j) = array_msg->data[i*des_length+j];
		}
	}

	char name[255];
	sprintf(name, "./submaps/%d_featureDes.map", des_idx);
	des_idx++;
	ofstream outD(name);
	outD<<"feature num:  "<<fea_num<<endl;
	outD<<"des length:  "<<des_length<<endl;
	outD<<"Data:  "<<feature_des<<endl;
	outD.close();

	FEATURE_DES_READY = true;
	printf("RECEIVED feature des done! \n");

}

void cloudCallback(pointcloud_type cloud_msg)
{

	printf("RECEIVED cloud: %d \n", cloud_msg.size());

	submap_pcd = pointcloud_type::Ptr (new pointcloud_type);
	pcl::copyPointCloud(cloud_msg, *submap_pcd);

	char name[255];
	sprintf(name, "./submaps/%d_cloud.pcd", cld_idx);
	cld_idx++;
	pcl::io::savePCDFile(name,*submap_pcd);

	FEATURE_CLOUD_READY = true;
	printf("RECEIVED cloud done! \n");
}
int main(int argc, char* argv[])
{

	ros::init(argc, argv, "mainMapBuilderROS");
    ParameterServer* ps = ParameterServer::instance();
    ParamSrvMi* mips = ParamSrvMi::instanceMi();

	ros::NodeHandle nh;
	ros::Subscriber ubmap_feature_loc_pub_ = nh.subscribe<std_msgs::Float32MultiArray>(mips->get<string>("submap_feature_loc_out_topic"), ps->get<int>("publisher_queue_size"),featureLocCallback);
	ros::Subscriber submap_feature_des_pub_ = nh.subscribe<std_msgs::Float32MultiArray>(mips->get<string>("submap_feature_des_out_topic"), ps->get<int>("publisher_queue_size"),featureDesCallback);
	ros::Subscriber submap_cloud_pub_ = nh.subscribe<pointcloud_type>(mips->get<string>("submap_cloud_out_topic"), ps->get<int>("publisher_queue_size"),cloudCallback);

	tf::TransformListener listener;
	int traj_idx =0;
	char name[255];

    CSubmapGraph graph_mgr;

	while(nh.ok())
	{
		if(FEATURE_LOC_READY && FEATURE_DES_READY && FEATURE_CLOUD_READY)
		{
			ROS_ERROR("mainMapBuilderROS: DATA READY");

			tf::StampedTransform transform;
			try{
				listener.lookupTransform("/submap/world", "/submap/cam",
						ros::Time(0), transform);
			}
			catch (tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
			}

			//build submap
			CSubmap<CNodeWrapper> * ps = new CSubmap<CNodeWrapper>;
			ps->m_root = transform;
			ps->id_ = traj_idx;
			ps->m_stamp = transform.stamp_;

			//add feature loc
			for(int i=0; i<feature_loc.rows; i++)
			{
				Eigen::Vector4f fea_loc;
				fea_loc(0) = feature_loc.at<float>(i,0);
				fea_loc(1) = feature_loc.at<float>(i,1);
				fea_loc(2) = feature_loc.at<float>(i,2);
				fea_loc(3) = 1.;
				ps->m_feature_locs.push_back(fea_loc);
			}

			//add feature descriptors
		    ps->m_feature_des = cv::Mat(feature_des);

		    //add point cloud
		    pcl::copyPointCloud(*submap_pcd, *(ps->pc_col));

		    CSubmapNode* node = new CSubmapNode(ps);
		    graph_mgr.addNode(node);
		   // if(!(traj_idx%5))
		    {
		    	ROS_ERROR("mainMapBuilderROS: START OPT NOW");
		    	sprintf(name, "./submaps/%d_preCloud.pcd", traj_idx);
		        graph_mgr.saveCloud2File(name);
		    	graph_mgr.optimizeSubmapGraph();
		    	sprintf(name, "./submaps/%d_posCloud.pcd", traj_idx);
		        graph_mgr.saveCloud2File(name);
		    }

			sprintf(name, "./submaps/%d_traj.txt", traj_idx);
			traj_idx++;
			ofstream outT(name);
			outT<<"Tf:  "<<transform<<endl;
			outT.close();

			FEATURE_DES_READY = false;
			FEATURE_LOC_READY = false;
			FEATURE_CLOUD_READY = false;
			ROS_ERROR("mainMapBuilderROS: ADD FINISHED!");
		}
		else
		{
			usleep(1000000);
			ros::spinOnce();

		}

	}

	//ros::spin();
    return 0;
}
