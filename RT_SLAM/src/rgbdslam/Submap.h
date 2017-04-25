#ifndef SUBMAP_H
#define SUBMAP_H

#include <iostream>
#include <vector>
#include "node.h"
#include "ros/ros.h"
#include "tf/tf.h"
#include "Eigen/Core"
#include "pcl/point_types.h"
#include "pcl/filters/passthrough.h"
#include "pcl/io/pcd_io.h"
#include "NodeWrapper.h"

#include "GraphWrapper.h"
#include "../octowrapper/ColorOctreeImpl.h"
#include "misc.h"
#include "SubmapStore.h"
#include <pcl_ros/transforms.h>
#include "slam_globaldef.h"

#define POINTCLOUD_ENABLED

// #include "../octowrapper/ColorOctreeImpl.h"
namespace submap
{
    typedef pcl::PointXYZRGB pt_type;
    // typedef pcl::PointXYZ    pt_type;
}
class GraphManager;
class ColorOctreeImpl;
using namespace std;

typedef std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > std_vector_of_eigen_vector4f;

template<typename NODE>
class CSubmap 
{
public:
    CSubmap();
    ~CSubmap();
    void addNode(vector<NODE*>& );
    void addNode(NODE*);
    void addFeatureIndex(NODE*);
    void addLandmarks(GraphManager*);
    void saveGraph(GraphManager* );
    void reduction();
    void resetSubmap();

    void print(string fname){ ofstream outf(fname.c_str()); if(outf.is_open()) print(outf);}
    void print(ostream& out);
    void dump2File(string fname="" );
    bool readFromFile(string fname);

    unsigned int id_;
    // 1 trajectory
    tf::StampedTransform base2points_;  
    tf::StampedTransform ground_truth_transform_; 
    tf::StampedTransform odom_transform_;        
    tf::Transform m_root;
    ros::Time m_stamp;
    vector<tf::Transform> m_traj;
    // 2 overlapped features
    
    // How to update feature? according to sigma of the feature points? 
    vector< NODE*> m_nodes;
    vector< vector<int> > m_feature_records; // the feature indexes for each node
    vector< vector<int> > m_node_records; // the node indexes in each feature
    
    std_vector_of_eigen_vector4f m_feature_locs; // locations for the features
    cv::Mat m_feature_des; // descriptors for the features

    // 3 point cloud
    bool m_bUseOctomap;
    ColorOctreeImpl* m_pOctoTree;   

    // reduction
    bool m_bHasReduced;
    // color_pc_ptr pc_col;
    // point_cloud_ptr pc_col;
    pcl::PointCloud<submap::pt_type>::Ptr pc_col;

    static unsigned int s_submap_store_id;
    static void setSubmapStoreId(unsigned int id);
private:
    CSubmap(const CSubmap&);
    CSubmap& operator=(const CSubmap&);
};

#include "Submap.hpp"

#endif
