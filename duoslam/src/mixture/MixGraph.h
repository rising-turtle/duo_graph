#ifndef MIX_GRAPH_H
#define MIX_GRAPH_H

#include <string>
#include <map>
#include <QList>

using namespace std;

#include <pcl_ros/transforms.h>
#include "g2o/core/sparse_optimizer.h"

// #include "g2o/types/slam3d/camera_parameters.h"
// #include "g2o/types/slam3d/parameter_camera.h"

// #include "g2o/core/hyper_dijkstra.h"
#include "g2o/core/robust_kernel_impl.h"
#include "matching_result.h"
#include "parameter_server.h"
#include "pcl/filters/voxel_grid.h"

class CSubmapNode;
class Node;
struct LoadedEdge3D;

class CMixGraph 
{
public:
    CMixGraph();
    virtual ~CMixGraph();
    bool addNode(CSubmapNode*);
    void updateGraph();
    void saveG2O(const char*);
    void getAllPC(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& );
    void getFeaturePC(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& );
    g2o::SparseOptimizer* getOptimizer(){return optimizer_;}
protected:
    void firstNode(CSubmapNode*);
    bool addEdgeToG2O(const LoadedEdge3D& edge, Node* n1, Node* n2, bool largeEdge, bool set_estimate);
    QList<int> bruteforceCompare(int ignore_nei);
    void createOptimizer(string backend, g2o::SparseOptimizer* pOptimizer=0);
    string m_back_end;
    MatchingResult curr_best_result_;
    map<int, CSubmapNode*> graph_;
    int next_vertex_id;
    g2o::HyperGraph::EdgeSet cam_cam_edges;
    g2o::SparseOptimizer* optimizer_;   
    g2o::RobustKernelHuber robust_kernel_;
};

namespace function
{
    template<typename PointT>
        void filter_PC(boost::shared_ptr< pcl::PointCloud<PointT> >& inPC, boost::shared_ptr< pcl::PointCloud<PointT> >& outPC)
        {
            float leaf_size = ParameterServer::instance()->get<double>("octomap_resolution");
            pcl::VoxelGrid<PointT> voxel_grid;
            voxel_grid.setInputCloud (inPC);
            voxel_grid.setLeafSize (leaf_size, leaf_size, leaf_size);
            voxel_grid.filter (*outPC);
        }

    template<typename PointT>
        void filter_PC(boost::shared_ptr< pcl::PointCloud<PointT> >& inPC)
        {
            float leaf_size = ParameterServer::instance()->get<double>("octomap_resolution");
            pcl::VoxelGrid<PointT> voxel_grid;
            voxel_grid.setInputCloud (inPC);
            voxel_grid.setLeafSize (leaf_size, leaf_size, leaf_size);
            boost::shared_ptr<pcl::PointCloud<PointT> > downsampled (new pcl::PointCloud<PointT>);
            voxel_grid.filter (*downsampled);
            (*inPC).swap(*downsampled);
        }
}


#endif
