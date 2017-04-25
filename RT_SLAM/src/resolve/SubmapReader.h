#ifndef SUBMAP_READER_H
#define SUBMAP_READER_H

#include <iostream>
#include <string>
#include <vector>
#include "Eigen/Core"
#include <opencv2/core/core.hpp>

using namespace std;

typedef std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > std_vector_of_eigen_vector4f;

namespace resolve{
typedef struct _Pose
{
    _Pose():tx(0),ty(0),tz(0),rx(0),ry(0),rz(0),rw(1){}
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

void print(ostream& out, resolve::Pose6d& p);
void print(ostream& out, resolve::SubmapHeader& h);
istream& operator>>(istream& in, resolve::Pose6d& p);
istream& operator>>(istream& in, resolve::SubmapHeader& h);

class CSubmapReader
{
public:
    CSubmapReader();
    ~CSubmapReader();
    bool read(string fname);    
    bool read(istream& );
    void print(string fname);
    void print(ostream&);
    size_t m_id; // id for this submap 
    double m_timestamp; // timestamp (seconds) for this submap
    resolve::Pose6d m_rootPose;
    vector<resolve::Pose6d> m_traj;
    vector< vector<int> > m_feature_records; // the feature indexes in each node
    vector< vector<int> > m_node_records; // the node indexes in each feature
    std_vector_of_eigen_vector4f m_feature_locs; // the 3D locations for each feature
    cv::Mat m_feature_des; // feature descriptors for the features
private:
    CSubmapReader(const CSubmapReader&);
    CSubmapReader operator=(const CSubmapReader&);
};


#endif
