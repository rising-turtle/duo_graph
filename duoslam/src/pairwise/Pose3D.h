#ifndef POSE3D_H
#define POSE3D_H

#include "tf/Transform.h"

namespace pairwise
{
class CPose3D
{
public: 
    CPose3D();
    CPose3D(double t[3], double q[4]);
    virtual ~CPose3D();
    tf::Transform pose2tf();
    void tf2pose(tf::Transform& );
private:
    double translation[3]; // xyz
    double quaternion[4]; // xyzw
};
}
#endif
