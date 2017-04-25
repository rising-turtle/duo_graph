#include "SubmapReader.h"
#include <fstream>

using namespace resolve;


void print(ostream& out, resolve::Pose6d& p)
{
     out<<"pose6d: "<<p.tx<<" "<<p.ty<<" "<<p.tz<<" "<<p.rx<<" "<<p.ry<<" "<<p.rz<<" "<<p.rw<<endl;
}
void print(ostream& out, resolve::SubmapHeader& h)
{
    out<<"id: "<<h.id<<" time: "<<h.timestamp<<endl;
    out<<"N_of_nodes: "<<h.n_nodes<<endl;
    out<<"N_of_feature: "<<h.n_features<<endl;
    out<<"root pose: "<<endl;
    print(out, h.root_pose);
}

istream& operator>>(istream& in, resolve::Pose6d& p)
{
    in>>p.tx>>p.ty>>p.tz>>p.rx>>p.ry>>p.rz>>p.rw;
    return in;
}
istream& operator>>(istream& in, resolve::SubmapHeader& h)
{
    in>>h.id>>h.timestamp>>h.n_nodes>>h.n_features;
    in>>h.root_pose;
    return in;
}


CSubmapReader::CSubmapReader(){}
CSubmapReader::~CSubmapReader(){}

bool CSubmapReader::read(string fname)
{
    ifstream inf(fname.c_str());
    if(!inf.is_open())
    {
        cerr<<"Error, fname: "<<fname<<" not exist!"<<endl;
        return false;
    }
    return read(inf);
}

bool CSubmapReader::read(istream& inf)
{
    // 1 read the header of the submap
    SubmapHeader header;
    inf>>header;
    m_rootPose = header.root_pose;
    m_timestamp = header.timestamp;
    m_id = header.id;

    // 2 read the trajectory
    cout<<"read header..."<<endl;
    ::print(cout, header);

    for(int i=0; i< header.n_nodes; i++)
    {
        Pose6d p;
        inf>>p;
        // tf::Transform tf_tr;
        // p.toTf(tf_tr);
        m_traj.push_back(p);
    }

    // 3 read the feature index of each node
    for(int i=0; i< header.n_nodes; i++)
    {
        // size_t M = m_feature_records[i].size(); 
        size_t M;
        inf>>M;
        vector<int> feature_index(M);
        for(int j=0;j<M;j++)
        {
            inf>>feature_index[j];
        }
        m_feature_records.push_back(feature_index);
    }
    
    // 4 dump the node index for each feature
    for(int i=0; i< header.n_features; i++)
    {
        // size_t M = m_node_records[i].size();
        size_t M;
        inf>>M;
        vector<int> node_index(M);
        for(int j=0;j<M;j++)
        {
            inf>>node_index[j];
        }
        m_node_records.push_back(node_index);
    }
     
    size_t N_fp = header.n_features;
    // outf<<N_fp;
    // 5 dump the feature locations 
    for(int i=0; i<N_fp; i++)
    {
        // Eigen::Vector4f& fea_loc = m_feature_locs[i];
        Eigen::Vector4f fea_loc;
        inf>>fea_loc(0)>>fea_loc(1)>>fea_loc(2); 
        fea_loc(3) = 1.;
        m_feature_locs.push_back(fea_loc);
    }

    // 6 dump the feature descriptors
    size_t feature_len;
    inf>>feature_len;
    m_feature_des = cv::Mat(N_fp, feature_len, CV_32FC1);
    for(int i=0; i<N_fp; i++)
    {
        for(int j=0;j<feature_len;j++)
        {
            // outf<<m_feature_des.at<float>(i,j);
            inf>>m_feature_des.at<float>(i,j);
        }
    }
    return true;
}

void CSubmapReader::print(string fname)
{
    ofstream ouf(fname.c_str());
    if(!ouf.is_open())
    {
        cerr<<"Error, file not exist: "<<fname<<endl;
        return ;
    }
    print(ouf);
}

void CSubmapReader::print(ostream& out)
{
    // 1 print the header of the submap
    out<<"Header of this submap: "<<endl;
    SubmapHeader header;
    header.id = m_id;
    header.timestamp = m_timestamp;
    header.n_nodes = m_feature_records.size();
    header.n_features = m_node_records.size();
    header.root_pose = m_rootPose;
    ::print(out, header);

    // 2 print the trajectory
    out<<"trajectory in this submap: "<<endl;
    for(int i=0; i< header.n_nodes; i++)
    {
        Pose6d p(m_traj[i]);
        ::print(out,p);
    } 
     
    // 3 print the feature index of each node
    out<<"feature index of each node: "<<endl;
    for(int i=0; i< header.n_nodes; i++)
    {
        size_t M = m_feature_records[i].size(); 
        out<<M<<": ";
        for(int j=0;j<M;j++)
        {
            out<<m_feature_records[i][j]<<" ";
        }
        out<<endl;
    }

    // 4 print the node index for each feature
    out<<"node index of each feature: "<<endl;
    for(int i=0; i< header.n_features; i++)
    {
        size_t M = m_node_records[i].size();
        out<<M<<": ";
        for(int j=0;j<M;j++)
        {
            out<<m_node_records[i][j]<<" ";
        }
        out<<endl;
    }

    size_t N_fp = header.n_features;
    // outf<<N_fp;
    // 5 print the feature locations 
    out<<"feature locations: "<<endl;
    for(int i=0; i<N_fp; i++)
    {
        Eigen::Vector4f& fea_loc = m_feature_locs[i];
        out<<fea_loc(0)<<" "<<fea_loc(1)<<" "<<fea_loc(2)<<endl; 
    }
    assert(N_fp == m_feature_des.rows);
    // 6 dump the feature descriptors
    out<<"feature descriptions: "<<endl;
    out<<"feature length: "<<m_feature_des.cols<<endl;
    for(int i=0; i<N_fp; i++)
    {
        for(int j=0;j<m_feature_des.cols;j++)
        {
            out<<m_feature_des.at<float>(i,j)<<" ";
        }
        out<<endl;
    }
}



