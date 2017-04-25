#ifndef SUBMAP_NODE_H
#define SUBMAP_NODE_H

#include <string>
#include "NodeWrapper.h"

using namespace std;
template<typename NODE>
class CSubmap;

class CSubmapNode : public CNodeWrapper
{
public:
    CSubmapNode(CSubmap<CNodeWrapper>* );
    CSubmapNode(string );
    virtual ~CSubmapNode();
    MatchingResult matchNodePair(CSubmapNode*);
    void init(CSubmap<CNodeWrapper>* );
    unsigned int node_id;
    tf::Transform node_root;
    double node_time;
    vector< vector<int> > m_feature_index;
    vector< vector<int> > m_node_index; 
    vector<tf::Transform> m_traj;
    void dump2File(string fname);
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


#endif
