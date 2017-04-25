#ifndef SLAM_GLOBALFUNC_H
#define SLAM_GLOBALFUNC_H

#include "Eigen/Core"
#include "tf/tf.h"

extern void fromTF2Eigen(const tf::transfrom& , Eigen::Matrix4f&);

#endif
