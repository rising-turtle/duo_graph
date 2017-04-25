#include "slam_globalfunc.h"

void fromTF2Eigen(tf::transfrom& tf_tran, Eigen::Matrix4f& matrix)
{
    matrix = Eigen::Matrix4f::getIdentity();
    tf::Vector3& tf_t = tf_tran.getOrigin();
    tf::Matrix3x3& tf_r = tf_tran.getBasis();
    matrix(0,3) = tf_t.getX(); matrix(1,3) = tf_t.getY(); matrix(2,3) = tf_t.getZ();
    for(int i=0;i<3;i++)
    {
        tf::Vector3& tf_tmp = tf_r[i];
        matrix(i,0) = tf_tmp.getX(); 
        matrix(i,1) = tf_tmp.getY();
        matrix(i,2) = tf_tmp.getZ();
    }
} 
