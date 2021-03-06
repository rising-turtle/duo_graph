#include "SubmapNode.h"
#include <pcl/filters/voxel_grid.h>
using namespace submap;

template<typename NODE>
unsigned int CSubmap<NODE>::s_submap_store_id = 0;

template<typename NODE>
CSubmap<NODE>::CSubmap():
    m_bHasReduced(false),
    // pc_col(new color_point_cloud),
    // pc_col(new pcl::PointCloud<pcl::PointXYZ>),
    pc_col(new pcl::PointCloud<submap::pt_type>),
    m_pOctoTree(new ColorOctreeImpl(ParameterServer::instance()->get<double>("octomap_resolution"))),
    // m_submap_store_id(0)
    id_(0),
    m_bUseOctomap(false) // Whether to use Octomap to filter the PointCloud
{}

template<typename NODE>
CSubmap<NODE>::~CSubmap()
{
    if(m_pOctoTree) delete m_pOctoTree;
    /*for(int i=0; i< m_nodes.size(); i++)
    {
        if(m_nodes[i] != 0)
        {
            delete m_nodes[i];
            m_nodes[i] = 0;
        }
    }*/
}

template<typename NODE>
void CSubmap<NODE>::setSubmapStoreId(unsigned int id)
{
    s_submap_store_id = id;
}

template<typename NODE>
void CSubmap<NODE>::addNode(vector<NODE*>& nodes)
{
    if(m_bHasReduced)
    {
        ROS_WARN("This submap has been reduced, reset? ");
        return ;
    }
    for(int i=0;i<nodes.size();i++)
        addNode(nodes[i]);
}
/*
template<typename NODE>
bool CSubmap<NODE>::isExisted(Eigen::Vector4f& query_pt, size_t query_id)
{
    if(query_id >= m_traj.size())
    {
        ROS_ERROR("query_id is bigger than traj.size()!");
        return false;
    }
    tf::Transform& tran = m_traj[query_id];
    Eigen::Matrix4f htrans;
    fromTF2Eigen(tran, htrans);
    for(int i=0;i<m_feature_locs.size();i++)
    {
        double dis_err = errorFunction2(query_pt, m_feature_locs[i], htrans);
        if(dis_err < gl_same_feature_dis)
        {
            // further to check the distance of descriptors
          // double des_err = 
        }
    }
    return true;
}*/

/*
template<typename NODE>
void CSubmap<NODE>::addFeatures(NODE* node_ptr, vector<int>& feature_index)
{
    int i = m_nodes.size();
    if(m_feature_index.size()==0) // frist node
    {
        int f_index = 0;
        // traverse each feature in node_ptr
        for(int j = 0; j<node_ptr->feature_descriptors_.rows; j++)
        {
            if(node_ptr->feature_matching_stats_[j] > gl_feature_observed_time)
            {  
                feature_index.push_back(f_index);
                ++f_index;
                m_feature_index.push_back(make_pair<int,int>(i,j));
                m_feature_locs.push_back(node_ptr->feature_locations_3d_[j]);
            }
        }
    }else
    {       
        size_t N = node_ptr->feature_descriptors_.rows;
        vector<bool> new_feature(N, false); 
        int result;
        for(int j=0; j<node_ptr->feature_descriptors_.rows; j++)
        {
            if(node_ptr->feature_matching_stats_[j] > gl_feature_observed_time)
            {
                Eigen::Vector4f pos = node_ptr->feature_locations_3d_[j];               
                // transform this feature
                if()
            }
        }
    }
}*/

template<typename NODE>
void CSubmap<NODE>::addFeatureIndex(NODE* node)
{
    vector<int> feature_index(node->kpt_to_landmark.size());
    map<int,int>::iterator it = node->kpt_to_landmark.begin();
    int index = 0;
    while(it!=node->kpt_to_landmark.end())
    {
        feature_index[index] = it->second;
        ++it;
        ++index;
    }
    m_feature_records.push_back(feature_index);
}

template<typename NODE>
void CSubmap<NODE>::addLandmarks(GraphManager* graph)
{
    CGraphWrapper * pg = static_cast<CGraphWrapper*>(graph);
    static unsigned int descriptor_length = pg->getFeatureLength(); // graph->descriptor_length;
    assert(descriptor_length != 0);
    int N = pg->getLandmarksNum();//pg->landmarks.size();
    ROS_WARN("number of landmarks: %i", N);
    // storage for all descriptors
    m_feature_des = cv::Mat( N, descriptor_length, CV_32FC1);
    Eigen::Matrix4f eigen_transform;

    CGraphWrapper::liter feature_it = pg->lbegin();
    int i = 0;
    while(feature_it!=pg->lend())
    {
        Landmark* lm = &(*feature_it); // &(graph->landmarks[i]);
        map<int,int>::iterator it = lm->observations.begin();
        NODE* pnode = m_nodes[it->first];
        // just use the first observed landmarks,    
        // TODO update the locations of the landmarks
        Eigen::Vector4f& loc_pt = m_nodes[it->first]->feature_locations_3d_[it->second];
        // transform this feature to the reference frame of root node
        tf::Transform& trans = m_traj[it->first];
        pcl_ros::transformAsMatrix(trans, eigen_transform);
        Eigen::Vector4f loc_trans = eigen_transform*loc_pt;
        m_feature_locs.push_back(loc_trans);
        
        // descriptions 
        cv::Mat feature_descs_ = pnode->feature_descriptors_;
        for(int j=0;j<descriptor_length;j++)
        {  
            // m_feature_des.at<float>(i,j) = (pnode->feature_descriptors_).at<float>(it->second,j);
            m_feature_des.at<float>(i,j) = feature_descs_.at<float>(it->second,j);
        }

        vector<int> nodes(lm->observations.size());
        int index = 0;
        while(it!=lm->observations.end())
        {
            nodes[index] = it->first;
            ++it;
            ++index;
        }
        m_node_records.push_back(nodes);
        // next loop
        ++feature_it;
        ++i;
    }    
    ROS_INFO("Finish adding landmarks!");
}


template<typename NODE>
void CSubmap<NODE>::addNode(NODE* node)
{
    static unsigned int ncout = 0;
    ROS_INFO("Submap.hpp start to addNode %d", ++ncout);
    if(m_bHasReduced) 
    {
        ROS_WARN("This submap has been reduced, reset ?!");
        return ;
    }
    // 1 trajectory
    if(m_traj.size()==0) // first Node
    {
        m_root = node->getBase2PointsTransform();// node->base2points_;
        id_ = node->id_;
        m_stamp = (node->getBase2PointsTransform()).stamp_;
    }
    ROS_INFO("add Trajectory");
    tf::Transform curr_pose(node->getBase2PointsTransform());

    // 2013.7.31 This is a big bug 
    // tf::Transform delta_motion = curr_pose * m_root.inverse();
    tf::Transform delta_motion = m_root.inverse() * curr_pose;

    m_traj.push_back(delta_motion);

    // 2 features
    ROS_INFO("add Feature Index");
    addFeatureIndex(node);   
    // addLandmarks();

    // 3 point cloud
    ROS_INFO("insert Point Cloud!");
    float p[7];
    submap::Pose6d tmpP(delta_motion);
    tmpP.outPut(p);

    // filter the point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> pass_through;
    pass_through.setInputCloud(node->pc_col);
    pass_through.setFilterFieldName("z");
    pass_through.setFilterLimits(ParameterServer::instance()->get<double>("minimum_depth"), ParameterServer::instance()->get<double>("maximum_depth"));
    pass_through.filter(*filtered_pc);
    ROS_INFO("after pass through PointCloud!");
    if(m_bUseOctomap)
    {
        // m_pOctoTree->insertPointCloud(*(node->pc_col),p);
        m_pOctoTree->insertPointCloud(*filtered_pc, p);
    }
    else
    {
        // not to use Octomap to filter the PointCloud
        transformAndAppendPointCloud(*filtered_pc, *pc_col, delta_motion, 6);
        // voxel filter this point cloud
        pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
        voxel_grid.setInputCloud (pc_col);
        float leaf_size = ParameterServer::instance()->get<double>("octomap_resolution");
        voxel_grid.setLeafSize (leaf_size, leaf_size, leaf_size);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled (new pcl::PointCloud<pcl::PointXYZRGB>);
        voxel_grid.filter (*downsampled);
        (*pc_col).swap(*downsampled);
    }
    ROS_INFO("after insert Point Cloud");
    // ROS_ERROR("Submap.hpp right: ADD TO PC_COL");
    // 4 record the pointer of this node
    m_nodes.push_back(node);
    ROS_INFO("Submap.hpp finish addNode %d", ncout);
    ROS_ERROR("Submap.hpp finish addNode %d", ncout);
}

template<typename NODE>
void CSubmap<NODE>::reduction()
{
    if(m_bHasReduced)
    {
        cout<<"This submap has been Reduced!"<<endl;
        return ;
    }
    m_bHasReduced = true;
    // 1 trajectory
    // 2 features
    // 3 point cloud
    int maxDepth = 16;
    
    // for(ColorOctreeImpl::leaf_iterator it = m_pOctoTree->begin(maxDepth), end=m_pOctoTree->end(); it!= end; ++it) 
    
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpPC(new pcl::PointCloud<pcl::PointXYZRGB>);
    if(m_bUseOctomap)
    {
        for(octomap::ColorOcTree::tree_iterator it = m_pOctoTree->begin_tree(maxDepth),
                end=m_pOctoTree->end_tree(); it!= end; ++it) 
        {
            if(it.isLeaf())
            {
                if(m_pOctoTree->isNodeOccupied(*it))
                {
                    octomap::point3d pt = it.getCoordinate();
                    octomap::ColorOcTreeNode::Color color = (*it).getColor();
                    pcl::PointXYZRGB color_pt(color.r, color.g, color.b);
                    color_pt.x = pt(0); color_pt.y = pt(1); color_pt.z = pt(2);
                    // tmpPC->points.push_back(color_pt);
                    pc_col->points.push_back(color_pt);
                    // pc_col->points.push_back(pcl::PointXYZ(pt(0),pt(1),pt(2)));
                    // list_iterator.push_back(OcTreeVolume(it.getCoordinate(), it.getSize()));
                }
            }
        }
        pc_col->width = pc_col->points.size();
        pc_col->height = 1;
    }

    // 4 delete all the data in the node
    /*for(int i=0;i<m_nodes.size();i++)
    {
        if(m_nodes[i] != 0)
        {
            delete m_nodes[i];
            m_nodes[i] = 0;
        }
    }*/
    {
        vector<NODE*> tmp;
        m_nodes.swap(tmp);
    }
}

template<typename NODE>
void CSubmap<NODE>::resetSubmap()
{
    if(m_bHasReduced == false)
    {
        m_bHasReduced = true;
    }
    // 1 trajectory
    {
        m_root.setIdentity();
        vector<tf::Transform> tmp;
        m_traj.swap(tmp);
    }
    // 2 features
    {
        // 2.1 feature & node index
        vector<vector<int> > tmp1;
        vector<vector<int> > tmp2;
        m_feature_records.swap(tmp1);
        m_node_records.swap(tmp2);
        
        // 2.2 feature locations
        std_vector_of_eigen_vector4f tmp3;
        m_feature_locs.swap(tmp3);

        // 2.3 feature descriptors
        // cv::Mat need no further consideration
    }
    // 3 nodes
    {
        if(m_nodes.size() > 0) 
        {
            vector<NODE*> tmp;
            m_nodes.swap(tmp);
        }
    }
    // 4 point cloud
    {
        // 4.1 PCL
        pcl::PointCloud<submap::pt_type>::Ptr tmp_pc(new pcl::PointCloud<submap::pt_type>);
        pc_col.swap(tmp_pc);
        
        // 4.2 Octomap
        if(m_pOctoTree !=0)
        {
            delete m_pOctoTree;
            m_pOctoTree = new ColorOctreeImpl;
        }
    }

    m_bHasReduced = false;
}

template<typename NODE>
void CSubmap<NODE>::saveGraph(GraphManager* pgraph)
{
    if(m_bHasReduced)
    {
        ROS_WARN("This submap has been reduced, reset? ");
        return ;
    }
    CGraphWrapper * pg = static_cast<CGraphWrapper*>(pgraph); 
    if(pg == 0)
    {
        ROS_ERROR("pgraph == 0! failed to save Graph");
        return ;
    }
    CGraphWrapper::giter it = pg->begin();
    Node* node = static_cast<Node*>(it->second);
    base2points_ = node->getBase2PointsTransform();// base2points_;
    ground_truth_transform_ = node->getGroundTruthTransform(); //ground_truth_transform_;
    odom_transform_ = node->getOdomTransform(); //odom_transform_;

    int ncout = 0;
    while(it != pg->end())
    {
        Node* pNode = static_cast<Node*>(it->second);
        if(!pNode->valid_tf_estimate_)
        {
            ROS_INFO("invalid_tf_estimate for this node");
            ++it;
            continue;
        }
        // set the tf::Transformation of this node
        g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(pg->getOptimizer()->vertex(pNode->vertex_id_));
        if(!v)
        { 
            ROS_ERROR("Nullpointer in graph at position %i!", it->first);
            ++it;
            continue;
        }
        tf::Transform tf_trans = eigenTransf2TF(v->estimate());
        tf::StampedTransform tf_base = pNode->getBase2PointsTransform();
        tf_base.setData(tf_trans);
        if(it == pg->begin())
        {
            submap::Pose6d firstPose(tf_trans);
            ROS_ERROR_STREAM("Submap.hpp in back graph firstPose: "<<firstPose);
        }
        // pNode->base2points_ = eigenTransf2TF(v->estimate());
        pNode->setBase2PointsTransform(tf_base);
        ROS_INFO("Start to add %i node", ncout+1);
        addNode(pNode);
        ROS_INFO("Finish add %i node", ncout+1);
        ROS_ERROR("Finish add %i node", ncout+1);
        ++ncout;
        ++it;
    }
    addLandmarks(pgraph);
    reduction();
    // 8 save g2o 
    {
        static unsigned int g2o_num = 0;
        stringstream ss;
        ss<<ParamSrvMi::instanceMi()->get<string>("submap_saved_path")<<"/"<<++g2o_num<<"g2o.log";
        pg->saveG2OGraph(ss.str().c_str());       
    }
}

template<typename NODE>
void CSubmap<NODE>::print(ostream& out)
{
    // 1 print the header of the submap
    out<<"Header of this submap: "<<endl;
    SubmapHeader header;
    header.id = id_; // s_submap_store_id;
    header.timestamp = m_stamp.toSec();
    header.n_nodes = m_feature_records.size();
    header.n_features = m_node_records.size();
    Pose6d root(m_root);
    header.root_pose = root;
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
    // 6 print the feature descriptors
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

template<typename NODE>
void CSubmap<NODE>::dump2File(string fname)
{
    string fn = fname;
    if(strcmp(fn.c_str(),"") == 0)
    {
        stringstream ss;
        ss<<s_submap_store_id<<"submap";
        ++s_submap_store_id;
        fn = ss.str();
    }
    // save all the
    stringstream ss;
    ss<<ParamSrvMi::instanceMi()->get<string>("submap_saved_path")<<"/"<<fn<<".map";
    ofstream outf(ss.str().c_str());
    
    // 1 dump the header of the submap
    SubmapHeader header;
    header.id = id_; //++m_submap_store_id;
    header.timestamp = m_stamp.toSec();
    header.n_nodes = m_feature_records.size();
    header.n_features = m_node_records.size();
    Pose6d root(m_root);
    header.root_pose = root;
    outf<<header;
    ROS_ERROR_STREAM("Submap.hpp header: "<<header);
    // 2 dump the trajectory
    for(int i=0; i< header.n_nodes; i++)
    {
        Pose6d p(m_traj[i]);
        // ROS_ERROR_STREAM("Submap.hpp traj"<<i<<" : "<<p);
        outf<<p;
    }

    // 3 dump the feature index of each node
    for(int i=0; i< header.n_nodes; i++)
    {
        size_t M = m_feature_records[i].size(); 
        outf<<M<<" ";
        for(int j=0;j<M;j++)
        {
            outf<<m_feature_records[i][j]<<" ";
        }
    }
    
    // 4 dump the node index for each feature
    for(int i=0; i< header.n_features; i++)
    {
        size_t M = m_node_records[i].size();
        outf<<M<<" ";
        for(int j=0;j<M;j++)
        {
            outf<<m_node_records[i][j]<<" ";
        }
    }
    
    size_t N_fp = header.n_features;
    // outf<<N_fp;
    // 5 dump the feature locations 
    for(int i=0; i<N_fp; i++)
    {
        Eigen::Vector4f& fea_loc = m_feature_locs[i];
        outf<<fea_loc(0)<<" "<<fea_loc(1)<<" "<<fea_loc(2)<<" "; 
    }
    assert(N_fp == m_feature_des.rows);
    // 6 dump the feature descriptors
    outf<<m_feature_des.cols<<" "; // write the length of the descriptors
    for(int i=0; i<N_fp; i++)
    {
        for(int j=0;j<m_feature_des.cols;j++)
        {
            outf<<m_feature_des.at<float>(i,j)<<" ";
        }
    }
#ifdef POINTCLOUD_ENABLED
    // 7 dump the pointcloud 
    // 7.1 ".pcd"
    {
        // save all the
        stringstream ss;
        ss<<ParamSrvMi::instanceMi()->get<string>("submap_saved_path")<<"/"<<fn<<".pcd";
        if(m_bUseOctomap)
        {
            pcl::io::savePCDFile(ss.str(),*pc_col);
        }else
        {
            ROS_ERROR("Submap.hpp Voxel_grid enabled! %f", ParameterServer::instance()->get<double>("octomap_resolution"));

        	ParamSrvMi* mips = ParamSrvMi::instanceMi();
        	if(mips->get<bool>("submap_downsample"))
        	{
                ROS_ERROR("Submap.hpp begin to save??");
        		pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
                if(pc_col->points.size() <=0 )
                {
                    voxel_grid.setInputCloud (pc_col);
                    float leaf_size = ParameterServer::instance()->get<double>("octomap_resolution");
                    voxel_grid.setLeafSize (leaf_size, leaf_size, leaf_size);
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled (new pcl::PointCloud<pcl::PointXYZRGB>);
                    voxel_grid.filter (*downsampled);
                    pcl::io::savePCDFile(ss.str(), *downsampled);
                }else{
                    ROS_ERROR("Submap.hpp: pc_col is empty!");
                }
        	}
        	else
        	{   
                ROS_ERROR("Submap.hpp begin to save file %s", ss.str().c_str());
        		pcl::io::savePCDFile(ss.str(), *pc_col);
        	}
        }
    }
    // 7.2 ".bt"
    {
        stringstream ss;
        ss<<ParamSrvMi::instanceMi()->get<string>("submap_saved_path")<<"/"<<fn<<".bt";
        if(m_bUseOctomap && m_pOctoTree->getNumLeafNodes() > 0)
            m_pOctoTree->writeBinary(ss.str());
    }
#endif

}

template<typename NODE>
bool CSubmap<NODE>::readFromFile(string fname)
{
    ifstream inf(fname.c_str());
    if(!inf.is_open())
    {
        ROS_ERROR("fname is not valid: %s!", fname.c_str());
        return false;
    }
    //TODO: checksum or flags to detect file type
    
    // 1 read the header of the submap
    SubmapHeader header;
    inf>>header;
    header.root_pose.toTf(m_root);
    m_stamp = ros::Time(header.timestamp);
    // m_submap_store_id = header.id;
    id_ = header.id;

    // 2 read the trajectory
    cout<<"read header..."<<endl;
    ::print(cout, header);

    for(int i=0; i< header.n_nodes; i++)
    {
        Pose6d p;
        inf>>p;
        tf::Transform tf_tr;
        p.toTf(tf_tr);
        m_traj.push_back(tf_tr);
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
    
    // 4 read the node index for each feature
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
    // 5 read the feature locations 
    for(int i=0; i<N_fp; i++)
    {
        // Eigen::Vector4f& fea_loc = m_feature_locs[i];
        Eigen::Vector4f fea_loc;
        inf>>fea_loc(0)>>fea_loc(1)>>fea_loc(2); 
        fea_loc(3) = 1.;
        m_feature_locs.push_back(fea_loc);
    }

    // 6 read the feature descriptors
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
#ifdef POINTCLOUD_ENABLED
    // 7 read the PointCloud
    // 7.1 ".pcd"
    {
        unsigned int npos = fname.find_last_of(".");
        string f1(fname, 0, npos);
        stringstream ss;
        ss<<f1<<".pcd";
        // ROS_ERROR("Submap.hpp: %s",ss.str().c_str());
        pcl::io::loadPCDFile(ss.str(), *pc_col);       
        if(pc_col->points.size() <= 0)
        {
            ROS_ERROR("Submap.hpp faild to load PointCloud!");
        }
    }
    // 7.2 ".bt"
    {
        if(m_bUseOctomap)
        {
            unsigned int npos = fname.find_last_of(".");
            string f1(fname, 0, npos);
            stringstream ss;
            ss<<f1<<".bt";
            ROS_WARN("read file %s", ss.str().c_str());
            //if(m_pOctoTree != 0) delete m_pOctoTree;
            // m_pOctoTree = new ColorOctreeImpl(ss.str());
            m_pOctoTree->readBinary(ss.str());
        }
    }
#endif
}
