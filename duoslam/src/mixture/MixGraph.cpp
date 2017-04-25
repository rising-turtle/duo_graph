#include "MixGraph.h"
#include "parameter_server.h"
#include "misc.h"
#include "edge.h"
#include <qtconcurrentrun.h>
#include <QtConcurrentMap>
#include <limits>

#include "g2o/types/slam3d/se3quat.h"
// #include "SubmapGraph.h"
#include "SubmapNode.h"
// #include "Submap.h"
#include "g2o/types/slam3d/edge_se3.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/core/optimization_algorithm_dogleg.h"
// #include <pcl_ros/transforms.h>

#include <pcl/point_cloud.h>

//typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
typedef g2o::BlockSolver< g2o::BlockSolverTraits<6, 3> >  SlamBlockSolver;
typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearCSparseSolver;
typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearCholmodSolver;
typedef g2o::LinearSolverPCG<SlamBlockSolver::PoseMatrixType> SlamLinearPCGSolver;
typedef g2o::LinearSolverDense<SlamBlockSolver::PoseMatrixType> SlamLinearDenseSolver;
//typedef std::map<int, g2o::VertexSE3*> VertexIDMap;
typedef std::tr1::unordered_map<int, g2o::HyperGraph::Vertex*>     VertexIDMap;
typedef std::pair<int, g2o::HyperGraph::Vertex*> VertexIDPair;
//std::tr1::unordered_map<int, g2o::HyperGraph::Vertex* >
typedef std::set<g2o::HyperGraph::Edge*> EdgeSet;
/*
namespace function
{
template<typename PointT>
void filter_PC(pcl::PointCloud<PointT>::Ptr& inPC, pcl::PointCloud<PointT>::Ptr& outPC)
{

}

template<typename PointT>
void filter_PC(pcl::PointCloud<PointT>::Ptr& inPC)
{

}

}
*/
CMixGraph::CMixGraph():
    optimizer_(0),
    m_back_end("cholmod"),
    next_vertex_id(0)
{
    createOptimizer(m_back_end);
}

CMixGraph::~CMixGraph()
{}

void CMixGraph::createOptimizer(string backend, g2o::SparseOptimizer* pOptimizer)
{
    // allocating the optimizer
    if(pOptimizer == NULL){
        if(optimizer_ != NULL){
            for(g2o::SparseOptimizer::VertexIDMap::iterator it = optimizer_->vertices().begin(); it != optimizer_->vertices().end(); it++)
            {
                it->second->edges().clear();
            }
            for(EdgeSet::iterator it = optimizer_->edges().begin(); it != optimizer_->edges().end(); it++)
            {
                //delete *it;
            }
            optimizer_->edges().clear();
            optimizer_->vertices().clear();
        }
        delete optimizer_;
        optimizer_ = new g2o::SparseOptimizer();
        optimizer_->setVerbose(true);
    } else if (optimizer_ != pOptimizer){
        delete optimizer_;
        optimizer_ = new g2o::SparseOptimizer();
        optimizer_->setVerbose(true);
    }
    SlamBlockSolver* solver = NULL;
    if(backend == "cholmod" || backend == "auto"){
        SlamLinearCholmodSolver* linearSolver = new SlamLinearCholmodSolver();
        linearSolver->setBlockOrdering(false);
        solver = new SlamBlockSolver(linearSolver);
        m_back_end = "cholmod";
    }
    else if(backend == "csparse"){
        SlamLinearCSparseSolver* linearSolver = new SlamLinearCSparseSolver();
        linearSolver->setBlockOrdering(false);
        solver = new SlamBlockSolver(linearSolver);
        m_back_end = "csparse";
    }
    else if(backend == "dense"){
        SlamLinearDenseSolver* linearSolver = new SlamLinearDenseSolver();
        solver = new SlamBlockSolver(linearSolver);
        m_back_end = "dense";
    }
    else if(backend == "pcg"){
        SlamLinearPCGSolver* linearSolver = new SlamLinearPCGSolver();
        solver = new SlamBlockSolver(linearSolver);
        m_back_end = "pcg";
    }
    else {
        ROS_ERROR("Bad Parameter for g2o Solver backend: %s. User cholmod, csparse or pcg", backend.c_str());
        ROS_INFO("Falling Back to Cholmod Solver");
        SlamLinearCholmodSolver* linearSolver = new SlamLinearCholmodSolver();
        linearSolver->setBlockOrdering(false);
        solver = new SlamBlockSolver(linearSolver);
        m_back_end = "cholmod";
    }
    //optimizer_->setSolver(solver);
    g2o::OptimizationAlgorithmDogleg * algo = new g2o::OptimizationAlgorithmDogleg(solver);
    optimizer_->setAlgorithm(algo);
}

void CMixGraph::firstNode(CSubmapNode* new_node)
{
    new_node->id_ = graph_.size();
    g2o::VertexSE3* reference_pose = new g2o::VertexSE3;

    new_node->vertex_id_ = next_vertex_id++;
    graph_[new_node->id_] = new_node;
    reference_pose->setId(new_node->vertex_id_);
    g2o::SE3Quat g2o_ref_se3; // = tf2G2O(init_base_pose_);
    reference_pose->setEstimate(g2o_ref_se3);
    reference_pose->setFixed(true);//fix at origin
    optimizer_->addVertex(reference_pose);
}

bool CMixGraph::addNode(CSubmapNode* new_node)
{
    if(graph_.size()==0)
    {
        firstNode(new_node);
        return true;
    }
    ROS_INFO("CMixGraph.cpp: new_node has %d features!", new_node->feature_locations_3d_.size());
    // 1 add sequential nodes
    new_node->id_ = graph_.size();
    // new_node->seq_id_ = next_seq_id++; // allways incremented, even if node is not added
    // unsigned int num_edges_before = cam_cam_edges.size();
    unsigned int num_edges_before = graph_.size();

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
    static const double seq_cov = 100;
    mr.edge.informationMatrix = Eigen::Matrix<double,6,6>::Identity()* seq_cov;//(mr.inlier_matches.size()/(mr.rmse*mr.rmse));
    // QMatrix4x4 curr_motion_estimate;
    if (addEdgeToG2O(mr.edge, last_node, new_node,  true, true)) 
    {
        graph_[new_node->id_] = new_node; //Needs to be added
        curr_best_result_ = mr;
    } else 
    {
        return false;
    }
    // 2 match with previous node 
    // CSubmapNode* last_node = graph_[graph_.size()-2];
    MatchingResult mr2 = new_node->matchNodePair(last_node);
    ROS_ERROR("new_node: id: %d, mr2. id1: %d, id2: %d, inliers: %d", new_node->id_, mr2.edge.id1, mr2.edge.id2, mr2.inlier_matches.size());
    if(mr2.edge.id1 >=0 )
    {
        //ROS_ERROR("MixGraph.cpp: succeed to match node %d with new_node!", last_node->vertex_id_);
        addEdgeToG2O(mr2.edge, graph_[mr2.edge.id1], new_node, false, false);
    }
    
    // 3 find loops
    QList<int> vertices_to_comp;
    ParameterServer* ps = ParameterServer::instance();
    
    // to fully enable loop_closure detector
    // brute forcely check loop match node 
    QList<const CSubmapNode* > nodes_to_comp;//only necessary for parallel computation
    int geod_cand = ps->get<int>("neighbor_candidates");
    vertices_to_comp = bruteforceCompare(geod_cand); // = getPotentialEdgeTargetsWithDijkstra(new_node, seq_cand, geod_cand, samp_cand, curr_best_result_.edge.id1); 

    for (int id_of_id = (int) vertices_to_comp.size() - 1; id_of_id >= 0; id_of_id--) 
    {
        CSubmapNode* pg = static_cast<CSubmapNode*>(graph_[vertices_to_comp[id_of_id]]);
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
        if (mr.edge.id1 >= 0 && mr.inlier_matches.size() > ParamSrvMi::instanceMi()->get<int>("submap_min_matches") ) 
        {
            assert(graph_[mr.edge.id1]);
            static const int loop_cov = 100; 
            mr.edge.informationMatrix += Eigen::Matrix<double,6,6>::Identity()* loop_cov;//(mr.inlier_matches.size()/(mr.rmse*mr.rmse));

            if ( addEdgeToG2O(mr.edge,graph_[mr.edge.id1],new_node, isBigTrafo(mr.edge.mean), (mr.inlier_matches.size() > curr_best_result_.inlier_matches.size() || (mr.edge.id1 + 1 == mr.edge.id2))))
            { 
                graph_[mr.edge.id1]->valid_tf_estimate_ = true;
                ROS_ERROR("Added Edge between %i and %i. Inliers: %i",mr.edge.id1,mr.edge.id2,(int) mr.inlier_matches.size());
                if (mr.inlier_matches.size() > curr_best_result_.inlier_matches.size()) 
                {
                    curr_best_result_ = mr;
                }
            }
        }
    }

    return graph_.size() > num_edges_before; 
}

bool CMixGraph::addEdgeToG2O(const LoadedEdge3D& edge, Node* n1, Node* n2, bool largeEdge, bool set_estimate)
{
    assert(n1);
    assert(n2);
    assert(n1->id_ == edge.id1);
    assert(n2->id_ == edge.id2);

    g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(n1->vertex_id_));
    g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(n2->vertex_id_));

    // at least one vertex has to be created, assert that the transformation
    // is large enough to avoid to many vertices on the same spot
    if (!v1 || !v2){
        if (!largeEdge) {
            ROS_INFO("Edge to new vertex is to short, vertex will not be inserted");
            return false;
        }
    }

    if(!v1 && !v2){
        ROS_ERROR("Missing both vertices: %i, %i, cannot create edge", edge.id1, edge.id2);
        return false;
    }
    else if (!v1 && v2) {
        v1 = new g2o::VertexSE3;
        assert(v1);
        int v_id = next_vertex_id++;
        v1->setId(v_id);

        n1->vertex_id_ = v_id; // save vertex id in node so that it can find its vertex
        v1->setEstimate(v2->estimateAsSE3Quat() * edge.mean.inverse());
        optimizer_->addVertex(v1);
        // motion_estimate = eigenTF2QMatrix(v1->estimate());
        ROS_WARN("Creating previous id. This is unexpected by the programmer");
    }
    else if (!v2 && v1) {
        v2 = new g2o::VertexSE3;
        assert(v2);
        int v_id = next_vertex_id++;
        v2->setId(v_id);
        n2->vertex_id_ = v_id;
        v2->setEstimate(v1->estimateAsSE3Quat() * edge.mean);
        // camera_vertices.insert(v2);
        optimizer_->addVertex(v2);
        // motion_estimate = g2o2QMatrix(v2->estimateAsSE3Quat());
    }
    else if(set_estimate){
        v2->setEstimate(v1->estimateAsSE3Quat() * edge.mean);
        // motion_estimate = g2o2QMatrix(v2->estimateAsSE3Quat());
    }
    g2o::EdgeSE3* g2o_edge = new g2o::EdgeSE3;
    g2o_edge->vertices()[0] = v1;
    g2o_edge->vertices()[1] = v2;
    g2o::SE3Quat meancopy(edge.mean);
    g2o_edge->setMeasurement(meancopy);
    //Change setting from which mahal distance the robust kernel is used: robust_kernel_.setDelta(1.0);
    g2o_edge->setRobustKernel(&robust_kernel_);
    // g2o_edge->setInverseMeasurement(edge.mean.inverse());
    g2o_edge->setInformation(edge.informationMatrix);
    optimizer_->addEdge(g2o_edge);
    ROS_DEBUG_STREAM("Added Edge ("<< edge.id1 << "-" << edge.id2 << ") to Optimizer:\n" << edge.mean.to_homogeneous_matrix() << "\nInformation Matrix:\n" << edge.informationMatrix);
    cam_cam_edges.insert(g2o_edge);
    // current_match_edges_.insert(g2o_edge); //Used if all previous vertices are fixed ("pose_relative_to" == "all")
}

QList<int> CMixGraph::bruteforceCompare(int ignore_nei)
{
    QList<int> ret; 
    if(graph_.size() <= ignore_nei)
    {
        return ret; 
    }
    map<int, CSubmapNode*>::iterator it = graph_.begin();
    for(; it!=graph_.end(); it++)
    {
        if(it->first + ignore_nei >= graph_.size()) 
            break;
        ret.push_front(it->first); 
    }
    return ret;
}

void CMixGraph::updateGraph()
{
    double stop_cond = ParameterServer::instance()->get<double>("optimizer_iterations");
    double chi2 = std::numeric_limits<double>::max();
    double pre_chi2;
    optimizer_->initializeOptimization(cam_cam_edges);
    do{
        pre_chi2 = chi2; 
        optimizer_->optimize(1);
        optimizer_->computeActiveErrors();
        chi2 = optimizer_->chi2();
        ROS_ERROR("MixGraph.cpp: pre_chi2: %lf, chi2: %lf", pre_chi2, chi2);
    }while((chi2/pre_chi2) < (1. - stop_cond));
}

void CMixGraph::getAllPC(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& outPC)
{
    map<int, CSubmapNode*>::iterator it = graph_.begin();
    while(it!=graph_.end())
    {
        CSubmapNode* node = it->second;
        g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(node->vertex_id_));
        tf::Transform tf = eigenTransf2TF(v->estimate());
        transformAndAppendPointCloud(*(node->pc_col), *outPC, tf, 20);
        function::filter_PC(outPC);
        it++;
    }
}

void CMixGraph::saveG2O(const char* fname)
{
    optimizer_->save(fname);
}

void CMixGraph::getFeaturePC(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& feat_PC)
{
    // pointcloud_type aggregate_cloud; ///will hold all other clouds
    // pcl::PointCloud<pcl::PointXYZRGB> aggregate_cloud; ///will hold all other clouds
    bool firstnode = true;
    map<int, CSubmapNode*>::iterator it = graph_.begin();
    for (; it != graph_.end(); ++it)
    {
        CSubmapNode* node = it->second;
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

        // tf::Transform transform = cam2rgb * world2cam.inverse(); // eigenTransf2TF(v->estimate());
        tf::Transform transform = eigenTransf2TF(v->estimate());
        transformAndAppendPointCloud (*feature_pc, *feat_PC, transform, ParameterServer::instance()->get<double>("maximum_depth"));
    }
    feat_PC->header.frame_id = "/feature_points";//base_frame;
}
