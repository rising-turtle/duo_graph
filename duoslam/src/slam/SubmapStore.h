#ifndef SUBMAP_STORE_H
#define SUBMAP_STORE_H

#include "tf/tf.h"
#include <iostream>

using namespace std;
namespace submap
{

typedef struct _Pose{
    _Pose(tf::Transform& tf_tr)
    {
        tf::Vector3 translation = tf_tr.getOrigin();
        tf::Quaternion quaternion = tf_tr.getRotation();
        tx = translation.getX(); ty = translation.getY(); tz = translation.getZ();
        rx = quaternion.getX(); ry = quaternion.getY(); rz = quaternion.getZ(); rw = quaternion.getW();
    }
    _Pose():tx(0),ty(0),tz(0),rx(0),ry(0),rz(0),rw(1){}
    void toTf(tf::Transform& tf_tr)
    {
        tf::Vector3 translation(tx,ty,tz);
        tf::Quaternion quaternion(rx,ry,rz,rw);
        tf_tr.setOrigin(translation);
        tf_tr.setRotation(quaternion);
    }
    void outPut(float p[7]){p[0] = tx; p[1] = ty; p[2] = tz; p[3] = rw; p[4] = rx; p[5] = ry; p[6] = rz;}
    float tx; // Translation
    float ty; 
    float tz;
    float rx; // Quaternion
    float ry;
    float rz;
    float rw;

}Pose6d;

typedef struct _SubmapHeader
{
    unsigned int id; // id for this submap
    double timestamp; // timestamp (sec) for this root node 
    // unsigned int timestamp;
    unsigned int n_nodes; // number of the nodes in the submap
    unsigned int n_features; // number of the features in the submap
    Pose6d root_pose; // root_pose of the submap
}SubmapHeader;
}

void print(ostream& out, submap::Pose6d& p);
void print(ostream& out, submap::SubmapHeader& h);
ostream& operator<<( ostream& out, const submap::Pose6d& p);
ostream& operator<<( ostream& out, const submap::SubmapHeader& h);
istream& operator>>(istream& in, submap::Pose6d& p);
istream& operator>>(istream& in, submap::SubmapHeader& h);

#endif
