#ifndef GRAPHHOLDER_H
#define GRAPHHOLDER_H

#include <camera_info_manager/camera_info_manager.h>
#include <string>
#include <tf/tf.h>
#include "NodeHolder.h"
using namespace std;

template <typename T >
tf::Transform eigenTransf2TF(const T& transf) 
{
    tf::Transform result;
    tf::Vector3 translation;
    translation.setX(transf.translation().x());
    translation.setY(transf.translation().y());
    translation.setZ(transf.translation().z());

    tf::Quaternion rotation;
    Eigen::Quaterniond quat;
    quat = transf.rotation();
    rotation.setX(quat.x());
    rotation.setY(quat.y());
    rotation.setZ(quat.z());
    rotation.setW(quat.w());

    result.setOrigin(translation);
    result.setRotation(rotation);
    //printTransform("from conversion", result);
    return result;
}

class CReadFiles;
// class CNodeHolder;

    typedef pcl::PointXYZRGB point_type;
    typedef pcl::PointCloud<point_type> pointcloud_type;
    typedef pointcloud_type::Ptr pointcloud_ptr;

class CGraphHolder
{
public:

    CGraphHolder(string dir);
    ~CGraphHolder();
    void runOffline(int step = 1, int num = -1, bool use_pc = false);
    void runOfflineGT(int step = 1, int num = -1, bool use_pc = false);
    void runOfflineGT2(int step = 1, int num = -1, bool use_pc = true);
protected:
    CNodeHolder* createNode(string, string);
    void parseGT(tf::Transform&, string);
    void dumpTrans2File(ostream& out, tf::Transform&, double);
private:
    tf::Transform eigen2TF(Eigen::Matrix4f& );
    void fusePC(pointcloud_ptr pc_all, pointcloud_ptr pc_append, tf::Transform);
    void setPCDPose(pointcloud_ptr pc, tf::Transform );
    sensor_msgs::CameraInfoPtr m_camInfo;
    CReadFiles * m_pRecords;
};

#endif
