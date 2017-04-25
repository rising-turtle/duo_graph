#include "Pose3D.h"

namespace pairwise
{
    CPose3D::CPose3D()
    {
        memset(translation, 0, sizeof(translation));
        memset(quaternion, 0, sizeof(quaternion));
        quaternion[3] = 1;
    }
    CPose3D::CPose3D(double t[3], double q[4])
    {
        memcpy(t, translation, 3*sizeof(double));
        memcpy(q, quaternion, 4*sizeof(double));
    }
    CPose3D::~CPose3D(){}

    tf::Transform CPose3D::pose2tf()
    {
        tf::Quaternion q(quaternion[0], quaternion[1], \
                        quaternion[2], quaternion[3]);
        tf::Vector3 t(translation[0], translation[1], translation[2]);
        return tf::Transform(q,t);
    }
    void CPose3D::tf2pose(tf::Transform& tf)
    {
        memcpy(*(tf.getOrigin()), &translation[0], 3*sizeof(double));
        memcpy(*(tf.getQuaternion()), &quaternion[0], 4*sizeof(double));
    }
}
