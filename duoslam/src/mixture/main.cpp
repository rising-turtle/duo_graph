#include <iostream>
#include <string>
#include <sstream>
#include "SubmapGraph.h"
#include "SubmapNode.h"
#include "Submap.h"
#include "MixGraph.h"

using namespace std;

void test1(); // test trajecory consistency
void test2(); // test features read
void test3(); // test specific submap match
void test4(); // test whether g2o works well

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "mixture");
    test1();
    return 0;
}

void test4()
{
    // ifstream inf("pos_g2o.log");
    CMixGraph graph; 
    graph.getOptimizer()->load("pos_g2o.log");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(int i=0; i< 72; i++)
    {
        g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(graph.getOptimizer()->vertex(i));
        tf::Transform tf = eigenTransf2TF(v->estimate());
        stringstream ss; 
        ss<<"./submaps"<<"/"<<i<<"submap.map";
        CSubmap<CNodeWrapper> * ps = new CSubmap<CNodeWrapper>;
        ps->readFromFile(ss.str());
        transformAndAppendPointCloud(*(ps->pc_col), *filtered_pc, tf, 6);
        function::filter_PC(filtered_pc);
    }
    pcl::io::savePCDFile("mixturePCDg2o.pcd", *filtered_pc);
}

void test3()
{
    ParamSrvMi* ps = ParamSrvMi::instanceMi();
    string prefix = ps->get<string>("submap_saved_path");
    
    stringstream ss1, ss2; 
    ss1<<prefix<<"/"<<0<<"submap.map";
    ss2<<prefix<<"/"<<28<<"submap.map";
    CSubmap<CNodeWrapper> * p1 = new CSubmap<CNodeWrapper>;
    p1->readFromFile(ss1.str());
    CSubmapNode* node1 = new CSubmapNode(p1);
    CSubmap<CNodeWrapper> * p2 = new CSubmap<CNodeWrapper>;
    p2->readFromFile(ss2.str());
    CSubmapNode* node2 = new CSubmapNode(p2);
    CMixGraph graph; 
    graph.addNode(node1);
    graph.addNode(node2);
    ROS_ERROR("main.cpp: finish add nodes!");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr fPC(new pcl::PointCloud<pcl::PointXYZRGB>);
    graph.getFeaturePC(fPC);
    pcl::io::savePCDFile("featurePC.pcd", *fPC);
}

void test2()
{
    ParamSrvMi* ps = ParamSrvMi::instanceMi();
    string prefix = ps->get<string>("submap_saved_path");
    stringstream ss; 
    ss<<prefix<<"/"<<108<<"submap.map";
    CSubmap<CNodeWrapper> * pS = new CSubmap<CNodeWrapper>;
    pS->readFromFile(ss.str());
    pS->dump2File("108submap2");
    /*CSubmapNode* node = new CSubmapNode(pS);
    CMixGraph graph; 
    graph.addNode(node);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr fPC(new pcl::PointCloud<pcl::PointXYZRGB>);
    graph.getFeaturePC(fPC);*/
    // pcl::io::savePCDFile("featurePC.pcd", *fPC);
}


void test1()
{
    ParamSrvMi* ps = ParamSrvMi::instanceMi();
    int n_first = ps->get<int>("submap_first");
    int n_maps = ps->get<int>("submap_number");
    int n_steps = ps->get<int>("submap_step");
    
    // string prefix("./submaps/");    
    string prefix = ps->get<string>("submap_saved_path");
    float leaf_size = ParameterServer::instance()->get<double>("octomap_resolution");
    /*
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
        for(int i=n_first; i<n_maps; i+=n_steps)
        {
            stringstream ss; 
            ss<<prefix<<"/"<<i<<"submap.map";
            CSubmap<CNodeWrapper> * ps = new CSubmap<CNodeWrapper>;
            ps->readFromFile(ss.str());
            tf::Transform tf;
            tf = ps->m_root;
            transformAndAppendPointCloud(*(ps->pc_col), *filtered_pc, tf, 6);
            function::filter_PC(filtered_pc);
        }
        pcl::io::savePCDFile("mixturePCDori.pcd", *filtered_pc);
    }
    */

    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_pc2(new pcl::PointCloud<pcl::PointXYZRGB>);

        CMixGraph mix_graph;

        for(int i=n_first; i<n_maps; i+=n_steps)
        {
            if(i >= 71 && i <= 72) continue;
            stringstream ss; 
            ss<<prefix<<"/"<<i<<"submap.map";
            CSubmap<CNodeWrapper> * ps = new CSubmap<CNodeWrapper>;
            ps->readFromFile(ss.str());
            CSubmapNode* node = new CSubmapNode(ps);
            mix_graph.addNode(node);
        }
        ROS_ERROR("after add_submaps!");
        mix_graph.saveG2O("pre_g2o.log");
        mix_graph.getAllPC(filtered_pc);
        pcl::io::savePCDFile("mixturePCDpre.pcd", *filtered_pc);
        mix_graph.updateGraph();
        mix_graph.getAllPC(filtered_pc2);
        pcl::io::savePCDFile("mixturePCDpos.pcd", *filtered_pc2);
    }
}


/*

        ps->readFromFile(ss.str());
        // transform into global reference
        tf::Transform tf; 
        tf = ps->m_root; 
       
        transformAndAppendPointCloud(*(ps->pc_col), *filtered_pc, tf, 6);
        pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
        voxel_grid.setInputCloud (filtered_pc);
        voxel_grid.setLeafSize (leaf_size, leaf_size, leaf_size);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled (new pcl::PointCloud<pcl::PointXYZRGB>);
        voxel_grid.filter (*downsampled);
        (*filtered_pc).swap(*downsampled);
*/
