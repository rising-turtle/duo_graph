#include "SubmapNode.h"
#include "Submap.h"

CSubmapNode::CSubmapNode(string fname)
{
    CSubmap<CNodeWrapper>* pSubmap = new CSubmap<CNodeWrapper>;
    pSubmap->readFromFile(fname);
    init(pSubmap);
}
CSubmapNode::CSubmapNode(CSubmap<CNodeWrapper>* pSubmap)
{
    init(pSubmap);
}
CSubmapNode::~CSubmapNode(){}
void CSubmapNode::init(CSubmap<CNodeWrapper>* pSubmap)
{
    // What a node need to complete matching?
    // 1 id 
    node_id = pSubmap->id_;//m_submap_store_id;
    id_ = -1; 
    seq_id_ = -1;
    vertex_id_ = -1;
    valid_tf_estimate_= true; 
    matchable_ = true;
    node_time = pSubmap->m_stamp.toSec();

    // feature & node indexes 
    for(int i=0; i< pSubmap->m_feature_records.size(); i++)
    {
        m_feature_index.push_back(pSubmap->m_feature_records[i]);
    }
    for(int i=0; i< pSubmap->m_node_records.size(); i++)
    {
        m_node_index.push_back(pSubmap->m_node_records[i]);
    }

    // 2 PointCloud
    pc_col = pSubmap->pc_col->makeShared();
    if(pc_col->points.size() <= 0)
    {
        ROS_ERROR("SubmapNode.cpp failed to init PointCloud from pSubmap");
    }

    // 3 trajectory
    node_root = pSubmap->m_root;
    base2points_ = pSubmap->base2points_;
    ground_truth_transform_ = pSubmap->ground_truth_transform_;
    odom_transform_ = pSubmap->odom_transform_;
    // whether the internal trajectory is necessary
    m_traj.insert(m_traj.begin(), pSubmap->m_traj.begin(), pSubmap->m_traj.end());
    
    // 4 features
    feature_locations_3d_.insert(feature_locations_3d_.begin(), 
        pSubmap->m_feature_locs.begin(), pSubmap->m_feature_locs.end());
    feature_descriptors_ = pSubmap->m_feature_des.clone();
    
    // note: hack the number of 2D features to enable featureMatching
    feature_locations_2d_.resize(feature_locations_3d_.size());

    // 5  flann 
    flannIndex = NULL;
}

MatchingResult CSubmapNode::matchNodePair(CSubmapNode* older_node)
{
    MatchingResult mr;
    ParameterServer* ps = ParameterServer::instance();
    if(ps->get<int>("max_connections") > 0 && initial_node_matches_ > ps->get<int>("max_connections")) 
    {
        return mr; //enough is enough
    }
    
    try{
        bool found_transformation = false;
        Node* node = static_cast<Node*>(older_node); 
        this->featureMatching(node, &mr.all_matches); 
        double ransac_quality = 0;
        if (mr.all_matches.size() < (unsigned int) ps->get<int>("min_matches"))
        {
            ROS_INFO("Too few inliers between %i and %i for RANSAC method. Only %i correspondences to begin with.",older_node->id_,this->id_,(int)mr.all_matches.size());
        }
        else 
        {//All good for feature based transformation estimation
            if(getRelativeTransformationTo(node, &mr.all_matches, mr.ransac_trafo, mr.rmse, mr.inlier_matches))
            {
                // TODO: further to refine the transformation using plane info? 
                found_transformation = true;
            }
            if(!found_transformation) mr.inlier_matches.clear();
        } 
        if(found_transformation) 
        {
            ROS_ERROR("SubmapNode.cpp succeed to match between %d and %d ", older_node->id_, this->id_);
            mr.edge.informationMatrix = Eigen::Matrix<double,6,6>::Identity()*(mr.inlier_matches.size()/(mr.rmse*mr.rmse));
            mr.final_trafo = mr.ransac_trafo;
            mr.edge.id1 = older_node->id_;
            mr.edge.id2 = this->id_;
            mr.edge.mean = eigen2G2O(mr.final_trafo.cast<double>());
            ++initial_node_matches_; //trafo is accepted
        } else 
        {
            // ROS_ERROR("SubmapNode.cpp failed to match between %d and %d ", older_node->id_, this->id_);
            mr.edge.id1 = mr.edge.id2 = -1;
        }
    }
    catch (std::exception e){//Catch exceptions: Unexpected problems shouldn't crash the application
        ROS_ERROR("Caught Exception in comparison of Nodes %i and %i: %s", this->id_, older_node->id_, e.what());
    }
    return mr;
}

void CSubmapNode::dump2File(string fname)
{
    CSubmap<CSubmapNode> * pSubmap = new CSubmap<CSubmapNode>;
    pSubmap->addNode(this);
    ROS_INFO("Start DUMP2File!");
    pSubmap->dump2File(fname);
    ROS_INFO("Finish DUMP2File!");
    delete pSubmap;
}

