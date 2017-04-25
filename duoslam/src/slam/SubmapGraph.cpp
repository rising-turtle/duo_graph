#include "SubmapGraph.h"
#include "SubmapNode.h"
#include "SubmapStore.h"
#include "misc.h"
#include <QThread>
#include <qtconcurrentrun.h>
#include <QtConcurrentMap> 
#include <pcl_ros/transforms.h>
#include "pcl/io/pcd_io.h"
#include "ColorOctreeImpl.h"

CSubmapGraph::CSubmapGraph(){}
CSubmapGraph::~CSubmapGraph(){}

bool CSubmapGraph::addNode(CSubmapNode* new_node)
{
    Node* pnode = static_cast<Node*>(new_node);
    if(graph_.size() == 0)
    {
        addFirstNode(pnode, new_node->node_root);
        return true;
    }

    // 1 add sequential nodes
    new_node->id_ = graph_.size();
    new_node->seq_id_ = next_seq_id++; // allways incremented, even if node is not added
    unsigned int num_edges_before = cam_cam_edges.size();
    bool edge_to_keyframe = false;

    CSubmapNode* last_node = dynamic_cast<CSubmapNode*>(graph_[graph_.size()-1]);
    //Get transform between submap nodes
    tf::Transform se_trans = last_node->node_root.inverse() * new_node->node_root;
    // tf::Transform se_trans = new_node->node_root * last_node->node_root.inverse();
    Eigen::Matrix4f eigen_transform;
    pcl_ros::transformAsMatrix(se_trans, eigen_transform);
    MatchingResult mr;
    mr.final_trafo = eigen_transform;
    mr.edge.id1 = last_node->id_;
    mr.edge.id2 = new_node->id_;
    mr.edge.mean = eigen2G2O(eigen_transform.cast<double>());
    // NOW: using the constant information matrix first 
    static const double seq_cov = 0.5; // 10
    mr.edge.informationMatrix = Eigen::Matrix<double,6,6>::Identity()* seq_cov;//(mr.inlier_matches.size()/(mr.rmse*mr.rmse));
    QMatrix4x4 curr_motion_estimate;
    if (addEdgeToG2O(mr.edge, last_node, new_node,  true, true, curr_motion_estimate)) 
    {
        graph_[new_node->id_] = new_node; //Needs to be added
        if(keyframe_ids_.contains(mr.edge.id1)) edge_to_keyframe = true;
        // updateInlierFeatures(mr, new_node, last_node);
        graph_[mr.edge.id1]->valid_tf_estimate_ = true;
        ROS_INFO("Added Edge between %i and %i. Inliers: %i",mr.edge.id1,mr.edge.id2,(int) mr.inlier_matches.size());
        curr_best_result_ = mr;
        //addOutliers(Node* new_node, mr.inlier_matches);
    } else 
    {
        process_node_runs_ = false;
        return false;
    }
    addKeyframe(new_node->id_-1);

    // 2 find loops
    QList<int> vertices_to_comp;
    ParameterServer* ps = ParameterServer::instance();
    int seq_cand = ps->get<int>("predecessor_candidates") - 1; 
    int geod_cand = ps->get<int>("neighbor_candidates");
    int samp_cand = ps->get<int>("min_sampled_candidates");
    // to fully enable loop_closure detector
    if(samp_cand < 20) samp_cand = 20; 
    QList<const CSubmapNode* > nodes_to_comp;//only necessary for parallel computation
    vertices_to_comp = getPotentialEdgeTargetsWithDijkstra(new_node, seq_cand, geod_cand, samp_cand, curr_best_result_.edge.id1); 

    for (int id_of_id = (int) vertices_to_comp.size() - 1; id_of_id >= 0; id_of_id--) 
    {
        CSubmapNode* pg = static_cast<CSubmapNode*>(graph_[vertices_to_comp[id_of_id]]);
        if(pg->id_ == new_node->id_ /*|| pg->id_ == new_node->id_-1*/) continue;
        // nodes_to_comp.push_front(graph_[vertices_to_comp[id_of_id]]); 
        nodes_to_comp.push_front(pg);
        ROS_INFO("Nodes to compare: %i", vertices_to_comp[id_of_id]);
    }
    
    QThreadPool* qtp = QThreadPool::globalInstance();
    ROS_INFO("Running node comparisons in parallel in %i (of %i) available threads", qtp->maxThreadCount() - qtp->activeThreadCount(), qtp->maxThreadCount());
    if (qtp->maxThreadCount() - qtp->activeThreadCount() == 1) 
    {
        //Never seen this problem...
        ROS_WARN("Few Threads Remaining: Increasing maxThreadCount to %i", qtp->maxThreadCount()+1);
        qtp->setMaxThreadCount(qtp->maxThreadCount() + 1);
    }
    QList<MatchingResult> results = QtConcurrent::blockingMapped(nodes_to_comp, boost::bind(&CSubmapNode::matchNodePair, new_node, _1));
    for (int i = 0; i < results.size(); i++) 
    {
        MatchingResult& mr = results[i];
        if (mr.edge.id1 >= 0 ) 
        {
            //mr.edge.informationMatrix *= geodesicDiscount(hypdij, mr);
            //ROS_INFO_STREAM("XY Information Matrix for Edge (" << mr.edge.id1 << "<->" << mr.edge.id2 << ") \n" << mr.edge.informationMatrix);
            ROS_ERROR("SubmapGraph.cpp begin to add edge in g2o with %d to %d", mr.edge.id1, mr.edge.id2);
            ROS_INFO("new node has id %i", new_node->id_);
            assert(graph_[mr.edge.id1]);
            CSubmapNode *gn = static_cast<CSubmapNode*>(graph_[mr.edge.id1]);
            ros::Duration delta_time(new_node->node_time - gn->node_time); //= new_node->pc_col->header.stamp - graph_[mr.edge.id1]->pc_col->header.stamp;
            /*if(!isSmallTrafo(mr.edge.mean, delta_time.toSec()))
            {
                ROS_ERROR("SubmapGraph.cpp small Trafo between %d to %d", mr.edge.id1, mr.edge.id2);
            }
            if(! addEdgeToG2O(mr.edge,graph_[mr.edge.id1],new_node, isBigTrafo(mr.edge.mean), mr.inlier_matches.size() > curr_best_result_.inlier_matches.size(), curr_motion_estimate))
            {
                ROS_ERROR("SubmapGraph.cpp failed to addEdgeToG2O!");
            }
            */
            if (/*isSmallTrafo(mr.edge.mean, delta_time.toSec()) &&*/
                mr.inlier_matches.size() > ParamSrvMi::instanceMi()->get<int>("submap_min_matches") &&
                    addEdgeToG2O(mr.edge,graph_[mr.edge.id1],new_node, isBigTrafo(mr.edge.mean), (mr.inlier_matches.size() > curr_best_result_.inlier_matches.size() || (mr.edge.id1 + 1 == mr.edge.id2)), curr_motion_estimate))
            { 
                graph_[new_node->id_] = new_node; //Needs to be added
                // updateInlierFeatures(mr, new_node, graph_[mr.edge.id1]);
                graph_[mr.edge.id1]->valid_tf_estimate_ = true;
                ROS_ERROR("Added Edge between %i and %i. Inliers: %i",mr.edge.id1,mr.edge.id2,(int) mr.inlier_matches.size());
                if (mr.inlier_matches.size() > curr_best_result_.inlier_matches.size()) 
                {
                    curr_best_result_ = mr;
                }
                if(keyframe_ids_.contains(mr.edge.id1)) edge_to_keyframe = true;
            }
        }
    }
    return cam_cam_edges.size() > num_edges_before; 
}

void CSubmapGraph::saveFeaturePosDes2File(string fname)
{

	std::ofstream ofs(fname.c_str());
	//cv::FileStorage fs(fname, cv::FileStorage::WRITE);

	tf::Transform firstNode2world;
	tf::Transform node2world;
	Eigen::Matrix4f eigen_transform;
	bool firstnode = true;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr feature_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
	int num_features=0;
	int len_feature=0;

	for (graph_it it = graph_.begin(); it != graph_.end(); ++it)
	{
		Node* node = it->second;
		if(!node->valid_tf_estimate_)
		{
			ROS_INFO("Skipping node %i: No valid estimate", node->id_);
			continue;
		}
		if(node->feature_locations_3d_.size() != node->feature_locations_2d_.size())
		{
			ROS_INFO("Skipping node %i: 2D size not equal 3D", node->id_);
			continue;
		}
		num_features +=node->feature_locations_3d_.size();
		if(!len_feature)
			len_feature = node->feature_descriptors_.cols;

	}
	ofs<<num_features<<" "<<len_feature<<endl;


	for (graph_it it = graph_.begin(); it != graph_.end(); ++it)
	{
		Node* node = it->second;
		if(!node->valid_tf_estimate_)
		{
			ROS_INFO("Skipping node %i: No valid estimate", node->id_);
			continue;
		}
		if(node->feature_locations_3d_.size() != node->feature_locations_2d_.size())
		{
			ROS_INFO("Skipping node %i: 2D size not equal 3D", node->id_);
			continue;
		}

		//node pose w.r.t to world
		g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(node->vertex_id_));
		if(!v)
		{
			ROS_ERROR("Nullpointer in graph at position %i!", it->first);
			continue;
		}
		if(firstnode)
		{
			firstNode2world = eigenTransf2TF(v->estimate());
			firstnode = false;
		}
		node2world = eigenTransf2TF(v->estimate());
		tf::Transform transform = firstNode2world.inverse() * node2world; // w.r.t the first node
		pcl_ros::transformAsMatrix(transform, eigen_transform);

		//transform land mark loc to global(first node here)
		for(int i=0; i< node->feature_locations_3d_.size(); i++)
		{
			Eigen::Vector4f& loc_pt = node->feature_locations_3d_[i];
			Eigen::Vector4f loc_trans = eigen_transform*loc_pt;

			//save locs of features
			ofs<<loc_trans.x()<<" "<<loc_trans.y()<<" "<<loc_trans.z();
			//save descriptors of features
			for(int j=0;j<node->feature_descriptors_.cols;j++)
			{
				ofs<<" "<<node->feature_descriptors_.at<float>(i, j);
			}
			ofs<<endl;

			//save feature cloud
			pcl::PointXYZRGB color_pt(255, 0, 0);
			color_pt.x = loc_trans.x(); color_pt.y = loc_trans.y(); color_pt.z = loc_trans.z();
			feature_pc->points.push_back(color_pt);
		}
	}

	ofs.close();
	pcl::io::savePCDFile("featurepcdtmp.pcd", *feature_pc, true); //Last arg is binary mode

}

void CSubmapGraph::saveFeature2File(string fname)
{
     // pointcloud_type aggregate_cloud; ///will hold all other clouds
    pcl::PointCloud<pcl::PointXYZRGB> aggregate_cloud; ///will hold all other clouds
    tf::Transform world2cam;
    tf::Transform cam2rgb;
    bool firstnode = true;
    for (graph_it it = graph_.begin(); it != graph_.end(); ++it)
    {
        Node* node = it->second;
        if(!node->valid_tf_estimate_) 
        {
            ROS_INFO("Skipping node %i: No valid estimate", node->id_);
            continue;
        }
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr feature_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
        for(int i=0; i< node->feature_locations_3d_.size(); i++)
        {
            Eigen::Vector4f& fpt = node->feature_locations_3d_[i];
            pcl::PointXYZRGB color_pt(255, 0, 0);
            color_pt.x = fpt(0); color_pt.y = fpt(1); color_pt.z = fpt(2);
            feature_pc->points.push_back(color_pt);
        }

        g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(node->vertex_id_));
        if(!v)
        { 
            ROS_ERROR("Nullpointer in graph at position %i!", it->first);
            continue;
        }
        if(firstnode)
        {
            world2cam = eigenTransf2TF(v->estimate());         
            firstnode = false;
        }
       
        cam2rgb = eigenTransf2TF(v->estimate());

        // 2013.7.31 This is a big bug 
        // tf::Transform transform = cam2rgb * world2cam.inverse(); // eigenTransf2TF(v->estimate());
        tf::Transform transform = world2cam.inverse() * cam2rgb;
        transformAndAppendPointCloud (*feature_pc, aggregate_cloud, transform, ParameterServer::instance()->get<double>("maximum_depth"));
    }
    aggregate_cloud.header.frame_id = "/feature_points";//base_frame;
    // if(filename.endsWith(".ply", Qt::CaseInsensitive))
        // pointCloud2MeshFile(filename, aggregate_cloud);
    pcl::io::savePCDFile(fname.c_str(), aggregate_cloud, true); //Last arg is binary mode
}

void CSubmapGraph::saveCloudByOctomap(string fname)
{
	ParameterServer* ps = ParameterServer::instance();
    ColorOctreeImpl* m_pOctoTree = new ColorOctreeImpl(ps->get<double>("octomap_resolution"));//gl_octomap_voxel_size);
    tf::Transform firstNode2world;
    tf::Transform node2world;
    // cam2rgb.setRotation(tf::createQuaternionFromRPY(-1.57,0,-1.57));
    // cam2rgb.setOrigin(tf::Point(0,-0.04,0));
    bool firstnode = true;
    for (graph_it it = graph_.begin(); it != graph_.end(); ++it)
    {
        Node* node = it->second;
        if(!node->valid_tf_estimate_) 
        {
            ROS_INFO("Skipping node %i: No valid estimate", node->id_);
            continue;
        }
        g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(node->vertex_id_));
        if(!v)
        { 
            ROS_ERROR("Nullpointer in graph at position %i!", it->first);
            continue;
        }
        if(firstnode)
        {
            firstNode2world = eigenTransf2TF(v->estimate());
            firstnode = false;
        }
       
        node2world = eigenTransf2TF(v->estimate());

        // 2013.7.31 This is a big bug 
        // tf::Transform transform = cam2rgb * world2cam.inverse(); // eigenTransf2TF(v->estimate());
        tf::Transform transform = firstNode2world.inverse() * node2world; // w.r.t the first node
        float p[7];
        submap::Pose6d tmpP(transform);
        tmpP.outPut(p);
        m_pOctoTree->insertPointCloud(*(node->pc_col), p);
        m_pOctoTree->updateInnerOccupancy();
        // transformAndAppendPointCloud (*(node->pc_col), aggregate_cloud, transform, ParameterServer::instance()->get<double>("maximum_depth"));
    }


    //save octomap
    //string filename = "octo.ot";
    std::ofstream outfile(fname.c_str(), std::ios_base::out | std::ios_base::binary);
    ROS_ERROR("Writing octomap to %s", fname.c_str());
    if (outfile.is_open()){
    	m_pOctoTree->write(outfile);
    }
    outfile.close();
    ROS_ERROR("color tree written %s", fname.c_str());

    /*
    pcl::PointCloud<pcl::PointXYZRGB> aggregate_cloud; ///will hold all other clouds
    int maxDepth = 16;
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
                aggregate_cloud.points.push_back(color_pt);
            }
        }
    }
    pcl::io::savePCDFile(fname.c_str(), aggregate_cloud, true); //Last arg is binary mode
    */
}

// save all the clouds into a single cloud
void CSubmapGraph::saveCloud2File(string fname)
{
    // pointcloud_type aggregate_cloud; ///will hold all other clouds
    pcl::PointCloud<pcl::PointXYZRGB> aggregate_cloud; ///will hold all other clouds

    // ROS_INFO("Saving all clouds to %s, this may take a while as they need to be transformed to a common coordinate frame.", qPrintable(filename));
    std::string base_frame  = ParameterServer::instance()->get<std::string>("base_frame_name");
    tf::Transform world2cam;
    tf::Transform cam2rgb;
    // cam2rgb.setRotation(tf::createQuaternionFromRPY(-1.57,0,-1.57));
    // cam2rgb.setOrigin(tf::Point(0,-0.04,0));
    bool firstnode = true;
    for (graph_it it = graph_.begin(); it != graph_.end(); ++it)
    {
        Node* node = it->second;
        if(!node->valid_tf_estimate_) 
        {
            ROS_INFO("Skipping node %i: No valid estimate", node->id_);
            continue;
        }
        g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(node->vertex_id_));
        if(!v)
        { 
            ROS_ERROR("Nullpointer in graph at position %i!", it->first);
            continue;
        }
        if(firstnode)
        {
            world2cam = eigenTransf2TF(v->estimate());         
            firstnode = false;
        }
       
        cam2rgb = eigenTransf2TF(v->estimate());

        // 2013.7.31 This is a big bug 
        // tf::Transform transform = cam2rgb * world2cam.inverse(); // eigenTransf2TF(v->estimate());
        tf::Transform transform = world2cam.inverse() * cam2rgb;

        transformAndAppendPointCloud (*(node->pc_col), aggregate_cloud, transform, ParameterServer::instance()->get<double>("maximum_depth"));
    }
    aggregate_cloud.header.frame_id = base_frame;
    // if(filename.endsWith(".ply", Qt::CaseInsensitive))
        // pointCloud2MeshFile(filename, aggregate_cloud);
    pcl::io::savePCDFile(fname.c_str(), aggregate_cloud, true); //Last arg is binary mode
}

void CSubmapGraph::saveRefinedSubmaps(string path)
{
    if(path==string(""))
    {
        path = "refined";
    }
    for (graph_it it = graph_.begin(); it != graph_.end(); ++it)
    {
        CSubmapNode* pn = static_cast<CSubmapNode*>(it->second);
        if(!pn->valid_tf_estimate_) 
        {
            ROS_INFO("Skipping node %i: No valid estimate", pn->id_);
            continue;
        }
        g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(pn->vertex_id_));
        stringstream ss ;
        ss<<path<<pn->id_<<"submap";
        pn->node_root = eigenTransf2TF(v->estimate());
        pn->dump2File(ss.str());
    }
}

void CSubmapGraph::optimizeSubmapGraph()
{
    ParameterServer* ps = ParameterServer::instance();
    ps->set<bool>("optimize_landmarks", false);
    optimizeGraph();
}



