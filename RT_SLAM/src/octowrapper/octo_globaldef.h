#ifndef OCO_GLOBALDEF_H
#define OCO_GLOBALDEF_H
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include "octomap/octomap_types.h"
#include "octomap/Pointcloud.h"
#include "gicp/gicp.h"
#include "FileReader.h"
#include "colortable.h"
#include "globaldef.h"

namespace octomap
{
    class Pointcloud;
}
namespace octowrapper
{
//typedef pcl::PointXYZRGB point_type;
typedef pcl::PointXYZ point_type;
// typedef pcl::PointXYZRGBA point_type;
typedef pcl::PointCloud<point_type> point_cloud;
typedef pcl::PointCloud<point_type>::Ptr point_cloud_ptr;
typedef pcl::PointCloud<point_type>::ConstPtr point_cloud_cptr;

typedef pcl::PointXYZRGBA color_point_type;
typedef pcl::PointCloud<color_point_type> color_point_cloud;
typedef pcl::PointCloud<color_point_type>::Ptr color_pc_ptr;
typedef pcl::PointCloud<color_point_type>::ConstPtr color_pc_cptr;
}

extern void readPose(const char*, octomap::pose6d&);

extern void fromPCL2OctoPC(octowrapper::point_cloud& , octomap::Pointcloud&, octomap::point3d& );
// template<typename PointT>
// void fromColorPCL2OctoPC(pcl::PointCloud<PointT>& , octomap::Pointcloud&, octomap::point3d&, vector<gl_color>&);
// extern void fromColorPCL2OctoPC(color_point_cloud&, octomap::Pointcloud&, octomap::point3d&, vector<gl_color>&);

extern dgc::gicp::GICPPointSet* fromPCL2GicpPC(octowrapper::point_cloud&);
extern void fromEigen2Pose6d(Eigen::Matrix4f& , octomap::pose6d&);
extern void fromPose6d2Eigen(Eigen::Matrix4f& , octomap::pose6d&);

template<typename PointT>
void readPcdAndPose(const char* fs, std::vector<boost::shared_ptr<pcl::PointCloud<PointT> > >& pcs, std::vector<octomap::pose6d>& poses, int num =-1, int step=5)
{
    CFileReader files;
    files.readDir(fs);
    string pcdf;
    string posef;
    octomap::pose6d pose;
    int tn;
    if(num<0) tn = files.m_lastnum;
    else tn = num < files.m_lastnum ? num : files.m_lastnum;
    for(int i=files.m_firstnum; i<= tn; i+=step)
    {
        boost::shared_ptr<pcl::PointCloud<PointT> > pc(new pcl::PointCloud<PointT>);
        // 1, get pcd & pose file
        files.getPoseFile(i, posef);
        files.getPcdFile(i, pcdf);
        // 2, get point cloud & pose6d 
        pcl::io::loadPCDFile(pcdf, *pc);
        pcs.push_back(pc);
        readPose(posef.c_str(), pose);
        poses.push_back(pose);
    }
}
template<typename PointT>
void fromColorPCL2OctoPC(pcl::PointCloud<PointT>& pcl_pc, octomap::Pointcloud& oct_pc, octomap::point3d& ori_pose, vector<gl_color>& color)
{
    octomap::point3d oc_pt;
    for(int i=0;i<pcl_pc.points.size();i++)
    {
        PointT& pt = pcl_pc.points[i];
        if(!pcl_isfinite(pt.x) || \
                !pcl_isfinite(pt.y) || \
                !pcl_isfinite(pt.z))
            continue;
        oc_pt(0) = pt.x;
        oc_pt(1) = pt.y;
        oc_pt(2) = pt.z;
        oct_pc.push_back(oc_pt);
        color.push_back(gl_color(pt.r,pt.g,pt.b));
    }   
    Eigen::Vector4f& _pose = pcl_pc.sensor_origin_;
    ori_pose(0) = _pose[0];
    ori_pose(1) = _pose[1];
    ori_pose(2) = _pose[2];
}

#endif
