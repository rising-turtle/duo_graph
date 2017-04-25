#include "octo_globaldef.h"

using namespace octowrapper;
void readPose(const char* fs, octomap::pose6d& pose)
{
    FILE* f = fopen(fs,"rb");
    if(f!=NULL)
    {
        double x,y,z,q0,q1,q2,q3;
        fscanf(f,"%lf %lf %lf %lf %lf %lf %lf",&x,&y,&z,&q0,&q1,&q2,&q3);
        pose = octomap::pose6d(octomath::Vector3(x,y,z), octomath::Quaternion(q0,q1,q2,q3));
        cout<<"read pose: "<<pose<<endl;
    }
    fclose(f);
}

void fromColorPCL2OctoPC(color_point_cloud& pcl_pc, octomap::Pointcloud& oct_pc, octomap::point3d& ori_pose, vector<gl_color>& color)
{
    octomap::point3d oc_pt;
    for(int i=0;i<pcl_pc.points.size();i++)
    {
        color_point_type& pt = pcl_pc.points[i];
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

void fromPCL2OctoPC(point_cloud& pcl_pc, octomap::Pointcloud& oct_pc, octomap::point3d& ori_pose)
{
    octomap::point3d oc_pt;
    for(int i=0;i<pcl_pc.points.size();i++)
    {
        point_type& pt = pcl_pc.points[i];
        if(!pcl_isfinite(pt.x) || \
                !pcl_isfinite(pt.y) || \
                !pcl_isfinite(pt.z))
            continue;
        oc_pt(0) = pt.x;
        oc_pt(1) = pt.y;
        oc_pt(2) = pt.z;
        oct_pc.push_back(oc_pt);
    }   
    Eigen::Vector4f& _pose = pcl_pc.sensor_origin_;
    ori_pose(0) = _pose[0];
    ori_pose(1) = _pose[1];
    ori_pose(2) = _pose[2];
}

void fromPose6d2Eigen(Eigen::Matrix4f& transformation, octomap::pose6d& pose)
{
    std::vector<double> rot(9);
    pose.rot().toRotMatrix(rot);
    for(int i=0;i<3;++i)
        for(int j=0;j<3;++j)
    {    
        transformation(i,j) = rot[i*3+j];
    }

    for(int i=0;i<3;i++)
        transformation(i,3) = pose.trans()(i);
    transformation(3,0) = transformation(3,1) = transformation(3,2) = 0;
    transformation(3,3) = 1;
}

void fromEigen2Pose6d(Eigen::Matrix4f& transformation, octomap::pose6d& pose)
{
    Eigen::Matrix3f m_ROT = transformation.block<3,3>(0, 0);
    Eigen::Vector3f translation = transformation.block<3,1>(0, 3); 
    // construct eular angle
    double roll,pitch,yaw;
    fromRot2RPY(roll,pitch,yaw,m_ROT);
    // construct translation x,y,z
    double x = translation(0); 
    double y = translation(1);
    double z = translation(2);
    pose = octomap::pose6d(x,y,z,roll,pitch,yaw);
}
