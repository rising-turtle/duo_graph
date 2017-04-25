#include "GraphWrapper.h"
#include "NodeWrapper.h"
#include "Submap.h"
#include "slam_globaldef.h"
// #include "ros/ros.h"

CGraphWrapper::CGraphWrapper():
   m_pSubmap(new CSubmap<CNodeWrapper>),
   m_bHasOptimized(false)
{
    optimizer_->setVerbose(false);
    m_first_pose.setIdentity();
}

CGraphWrapper::~CGraphWrapper()
{
    if(m_pSubmap != 0) delete m_pSubmap;
}

void CGraphWrapper::addFirstNode(Node* new_node, tf::Transform p)
{
    //set the node id only if the node is actually added to the graph
    //needs to be done here as the graph size can change inside this function
    new_node->id_ = graph_.size();
    // ROS_ERROR("GraphWrapper.cpp addFirstNode id: %d", new_node->id_);
    new_node->seq_id_ = next_seq_id++; // allways incremented, even if node is not added
    init_base_pose_ =  new_node->getGroundTruthTransform();//identity if no MoCap available
    printTransform("Ground Truth Transform for First Node", init_base_pose_);
    //new_node->buildFlannIndex(); // create index so that next nodes can use it
    
    m_first_pose = p;
    submap::Pose6d pp(p);
    ROS_ERROR_STREAM("GraphWrapper.cpp set the first node with pose: "<<pp);
    g2o::SE3Quat pose = tf2G2O(p);
    g2o::VertexSE3 ori_identity;
    g2o::VertexSE3* reference_pose = new g2o::VertexSE3;
    reference_pose->setEstimate(ori_identity.estimateAsSE3Quat() * pose);

    new_node->vertex_id_ = next_vertex_id++;
    graph_[new_node->id_] = new_node;
    reference_pose->setId(new_node->vertex_id_);

    camera_vertices.insert(reference_pose);

    // ROS_ERROR("Adding initial node with id %i and seq %i, v_id: %i", new_node->id_, new_node->seq_id_, new_node->vertex_id_);
    g2o::SE3Quat g2o_ref_se3 = tf2G2O(init_base_pose_);
    // reference_pose->setEstimate(g2o_ref_se3);
    reference_pose->setFixed(true);//fix at origin
    optimizer_mutex_.lock();
    optimizer_->addVertex(reference_pose); 
    optimizer_mutex_.unlock();

    g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(new_node->vertex_id_));
    tf::Transform tf_trans = eigenTransf2TF(v->estimate());
    submap::Pose6d ppp(tf_trans);
    ROS_ERROR_STREAM("GraphWrapper.cpp to check firstPose: "<<ppp);

    // QString message;
    // Q_EMIT setGUIInfo(message.sprintf("Added first node with %i keypoints to the graph", (int)new_node->feature_locations_2d_.size()));
    //pointcloud_type::Ptr the_pc(new_node->pc_col); //this would delete the cloud after the_pc gets out of scope
    QMatrix4x4 latest_transform = g2o2QMatrix(g2o_ref_se3);
    /*if(!ParameterServer::instance()->get<bool>("glwidget_without_clouds")) { 
        Q_EMIT setPointCloud(new_node->pc_col.get(), latest_transform);
        Q_EMIT setFeatures(&(new_node->feature_locations_3d_));
    }*/
    current_poses_.append(latest_transform);
    this->addKeyframe(new_node->id_);
    process_node_runs_ = false;
}

tf::Transform CGraphWrapper::getLastNodePose()
{
    if(!m_bHasOptimized)
    {
        optimizeGraph();
        m_bHasOptimized = true;
    }
    optimizer_mutex_.lock();
    g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(graph_[graph_.size()-1]->vertex_id_));
    tf::Transform previous = eigenTransf2TF(v->estimate());
    optimizer_mutex_.unlock();
    return previous;
}

void CGraphWrapper::reduceGraph()
{
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);
    ROS_ERROR("start to reduce Graph!");
    if(!m_bHasOptimized)
    {
        optimizeGraph();
        m_bHasOptimized = true;
    }
    m_pSubmap->saveGraph(this);
    m_pSubmap->reduction();
    m_pSubmap->dump2File();
    // string file("./submaps/prefile.log");
    // m_pSubmap->print(file);
    m_pSubmap->resetSubmap();
    clock_gettime(CLOCK_MONOTONIC, &end);
    double time_elapsed = (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) / 1e9;
    ROS_ERROR("finish reducing Graph, cost: %lf s", time_elapsed);
    resetGraph();
    optimizer_->setVerbose(false);
}

void CGraphWrapper::submapSwap()
{
    static bool once = true;
    if(once && graph_.size() > gl_submap_size)
    {
        ROS_WARN("Submap begin!");
        ROS_INFO("Start save Graph!");
        m_pSubmap->saveGraph(this);
        ROS_WARN("Finish save Graph!");
        ROS_INFO("Start reduction!");
        m_pSubmap->reduction();
        ROS_WARN("Finish save Graph!");
        ROS_INFO("Start dump2File!");
        m_pSubmap->dump2File();
        ROS_WARN("Finish dump 2File!");
        string file("./submaps/prefile.log");
        m_pSubmap->print(file);
        ROS_WARN("Finish print file");
        m_pSubmap->resetSubmap();
        ROS_INFO("Start resetGraph!");
        resetGraph();
        ROS_WARN("Finish resetGraph!");
        // 1, reset current graph
        // reset_request_ = true; // 
        // 2, send current graph info

        once = false;
    }      
}

void CGraphWrapper::resetGraph()
{
    ROS_ERROR("GraphWrapper.cpp in subclass resetGraph()!");
    GraphManager::resetGraph();
    setVerbose(false);
    m_bHasOptimized = false;
}

void CGraphWrapper::resetFirstPose()
{
    submap::Pose6d pp(m_first_pose);
    ROS_ERROR_STREAM("GraphWrapper.cpp reset the first node with pose: "<<pp);
    g2o::SE3Quat pose = tf2G2O(m_first_pose);
    g2o::VertexSE3 ori_identity;
    // reference_pose->setEstimate(ori_identity.estimateAsSE3Quat() * pose);
    g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(graph_[0]->vertex_id_));
    v->setEstimate(ori_identity.estimateAsSE3Quat() * pose);
    // tf::Transform tf_trans = eigenTransf2TF(v->estimate());
}
