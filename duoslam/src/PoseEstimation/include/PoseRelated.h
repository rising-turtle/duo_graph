/*
 * PoseRelated.h
 *
 *  Created on: Nov 4, 2013
 *      Author: helen
 */

#ifndef POSERELATED_H_
#define POSERELATED_H_
#include <EDPbmap.h>

void crossProduct3D(Eigen::Matrix<double,3,1> v0, Eigen::Matrix<double,3,1> v1, Eigen::Matrix<double,3,1> &v2);
void rodrigues_so3_exp(Eigen::Matrix<double,3,1> w, double A, double B, Eigen::Matrix<double,3,3> R);
void exp_HomogeneousMatrix(const Eigen::Matrix<double,6,1>& mu, Eigen::Matrix4f &ResultMatrix, bool pseudo_exponential = false);

#endif /* POSERELATED_H_ */
