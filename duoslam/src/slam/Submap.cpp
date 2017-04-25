#include "Submap.h"
/*#include "GraphWrapper.h"
#include "misc.h"
#include "SubmapStore.h"
#include <pcl_ros/transforms.h>
#include "slam_globaldef.h"
*/
using namespace submap;

template<>
void CSubmap<CSubmapNode>::addNode(CSubmapNode* node)
{
    ROS_INFO("Submap.cpp in the overloaded addNode()!");
    if(m_bHasReduced)
    {
        ROS_ERROR("This submap has been reduced, reset ?!");
        return ;
    }
    // 1 trajectory
    if(m_traj.size()==0)
    {
        m_root = node->node_root;
        id_ = node->node_id;
        // m_stamp = node->node_time;
        m_stamp = ros::Time(node->node_time);
        base2points_ = node->getBase2PointsTransform();
        ground_truth_transform_ = node->getGroundTruthTransform();
        odom_transform_ = node->getOdomTransform();
    }
    ROS_INFO("add Trajectory");
    /*for(int i=0; i< node->m_traj.size(); i++)
    {
        tf::Transform curr_pose(node->getBase2PointsTransform());
        tf::Transform delta_motion = curr_pose * m_root.inverse();
    }*/
    m_traj.insert(m_traj.begin(), node->m_traj.begin(), node->m_traj.end());
    
    // 2 feature index
    for(int i=0; i< node->m_feature_index.size(); i++)
    {
        m_feature_records.push_back(node->m_feature_index[i]);
    }
    for(int i=0; i< node->m_node_index.size(); i++)
    {
        m_node_records.push_back(node->m_node_index[i]);
    }
    // 3 features 
    m_feature_locs.insert(m_feature_locs.begin(), node->feature_locations_3d_.begin(),
            node->feature_locations_3d_.end());
    m_feature_des = node->feature_descriptors_.clone();

    // 4 point cloud
    pc_col = node->pc_col->makeShared();
    if(pc_col->size() <=0 )
    {
        ROS_ERROR("Submap.cpp failed to add PointCloud in addNode");
    }
}

