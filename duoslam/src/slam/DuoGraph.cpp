#include "DuoGraph.h"
#include "node.h"
#include "tf/tf.h"
#include <qtconcurrentrun.h>
#include "SubmapStore.h"

CDuoGraph::CDuoGraph() : 
m_mgr_fr(new CGraphWrapper),
m_mgr_bk(new CGraphWrapper),
m_bhas_switched(false)
{
    m_mgr_fr->setVerbose(false);
    m_mgr_fr->setDraw(true);
    m_mgr_bk->setVerbose(false);
    m_mgr_bk->setDraw(false);

    ParamSrvMi* mips = ParamSrvMi::instanceMi();

    if(mips->get<bool>("globalmap_online_socket"))
    {
    	char *pcIPClient=mips->get<string>("socket_client_ip").c_str();
    	int nPortClient=mips->get<int>("socket_client_port");

    	char *pcIPHost=mips->get<string>("socket_host_ip").c_str();
    	int nPortHost=mips->get<int>("socket_host_port");

    	char cClientConf[40];
    	memset(cClientConf,0,40);

    	memcpy(cClientConf,pcIPClient,strlen(pcIPClient));
    	memcpy(&cClientConf[16],&nPortClient,4);

    	memcpy(&cClientConf[20],pcIPHost,strlen(pcIPHost));
    	memcpy(&cClientConf[36],&nPortHost,4);

    	m_client.ClientNetInit(cClientConf);
    	ROS_ERROR("Init client finished! \n");
    }
}
CDuoGraph::~CDuoGraph(){}

bool CDuoGraph::addNode(Node* node_ptr)
{
    bool ret;
    static int n_fr = 0;

    ParamSrvMi* mips = ParamSrvMi::instanceMi();

    // 1 if need to start the back graph
    int start_dg_size = mips->get<int>("submap_size") - mips->get<int>("submap_overlap_size");
    if(m_mgr_fr->size() > start_dg_size)//gl_start_dg_size)
    {
        // ROS_ERROR_STREAM("DuoGraph.cpp start bk_mgr! fr_size: "<<m_mgr_fr->size());
        Node* node_bk = new Node(*node_ptr);
        ret = m_mgr_fr->addNode(node_ptr); 
        if(!ret)
        {
            delete node_bk;
            return ret;
        }
        else ROS_ERROR_STREAM("DuoGraph.cpp add Node in fr_mgr! fr_size: "<<m_mgr_fr->size());
        node_bk->id_ = node_ptr->id_;
        
        while(!future_.isFinished())
        {
            ROS_ERROR("That's the problem we are fear to happen!");
            future_.waitForFinished(); //Wait if back GraphManager ist still computing. 
        }
        
        if(m_mgr_bk->size() >= mips->get<int>("submap_size"))
        {
            if(future_.isFinished()){ROS_ERROR("Finished!");}
            if(future_.isPaused()) {ROS_ERROR("Paused!");}
            if(future_.isRunning()) {ROS_ERROR("Running!");}
            if(future_.isCanceled()) {ROS_ERROR("Canceled!");}
        }
        // the first node for the back graph
        if(m_mgr_bk->size()==0)
        {
            tf::Transform p = m_mgr_fr->getLastNodePose();
            submap::Pose6d pose(p);
            ROS_ERROR_STREAM("Last transpose: "<<pose);
            if(m_mgr_bk->IsDraw())
            {
                ROS_ERROR("DuoGraph.cpp back graph can draw!!!");
                m_mgr_bk->setDraw(false);
            }
            m_mgr_bk->addFirstNode(node_bk, p);
        }else
        {
            if(m_mgr_bk->IsDraw())
            {
                ROS_ERROR("DuoGraph.cpp back graph can draw!!!");
                m_mgr_bk->setDraw(false);
            }
            if(!m_mgr_bk->addNode(node_bk))
            {
                ROS_ERROR("Failed to add Node_back!!");
                delete node_bk;
            }
            else 
            {
                // 2013.7.31 This is a bug, when replace the first node
                // We have to reset the pose of the first vertex
                if(m_mgr_bk->size()==1)
                {
                    m_mgr_bk->resetFirstPose();    
                }
                ROS_ERROR_STREAM("DuoGraph.cpp add Node in bk_mgr! bk_size: "<<m_mgr_bk->size());
            }
            if(m_mgr_bk->getOptimizer()->verbose())
            {
                m_mgr_bk->setVerbose(false);
            }
        }
        // 2 if need to reduce into a submap
        // if(m_mgr_fr->size() >= gl_submap_size)
        if(m_mgr_bk->size() >= mips->get<int>("submap_overlap_size"))//gl_overlap_size)
        {
            ROS_ERROR_STREAM("DuoGraph.cpp swap happen! bk_size: "<<m_mgr_bk->size());
            ROS_INFO_STREAM("DuoGraph.cpp swap happen! fr_size: "<<m_mgr_fr->size());
            ROS_INFO_STREAM("DuoGraph.cpp swap happen! bk_size: "<<m_mgr_bk->size());
            m_mgr_fr.swap(m_mgr_bk);
            m_mgr_fr->setDraw(true);
            m_mgr_bk->setDraw(false);
            m_bhas_switched = true;
            future_ = QtConcurrent::run(m_mgr_bk.get(), &CGraphWrapper::reduceGraph, &m_client);
            m_mgr_fr->resetViewer();
            ROS_ERROR("DuoGraph.cpp resetView()!");
        }
    }else{
        ret = m_mgr_fr->addNode(node_ptr);
        if(ret)
            ROS_ERROR_STREAM("DuoGraph.cpp add node"<< ++n_fr<<" for the fr_graph:size() "<<m_mgr_fr->size());
        if(m_mgr_fr->getOptimizer()->verbose())
        {
            m_mgr_fr->setVerbose(false);
        }
    }
  return ret;
}


