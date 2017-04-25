#include "SubmapGraph.h"
#include "SubmapNode.h"
#include "Submap.h"
#include "qtros.h"
#include <QObject>
#include <QApplication>
#include <iostream>
#include <sstream>
#include <string>
#include "stdlib.h"

using namespace std;

int main(int argc, char* argv[])
{
    QtROS qtRos(argc, argv, "submap");
    //Depending an use_gui on the Parameter Server, a gui- or a headless application is used
    QApplication app(argc, argv, ParameterServer::instance()->get<bool>("use_gui")); 
    string path = "./submaps";

    int i;
    if(argc >= 2) 
    {
        i = atoi(argv[1]); //26;
        {
            stringstream ss;
            ss<<path<<"/"<<i<<"submap.map";
            CSubmap<CNodeWrapper> * ps = new CSubmap<CNodeWrapper>;
            ps->readFromFile(ss.str());
            CSubmapNode* node = new CSubmapNode(ps);
            string dumpf("after");
            if(argc >=3 ) dumpf = string(argv[2]);
            node->dump2File(dumpf.c_str());
            // graph_mgr.addNode(node);
        }
    }
    else 
    {
        ROS_INFO("Input number is %d", i);
        CSubmapGraph graph_mgr;
        int n = 18;
        ParameterServer * ps = ParameterServer::instance();
        ps->set<int>("ransac_iterations" , 200);
        ps->set<double>("max_dist_for_inliers", 5);
        for(int i=0; i< n; i++)
        {
            stringstream ss;
            ss<<path<<"/"<<i<<"submap.map";
            CSubmap<CNodeWrapper> * ps = new CSubmap<CNodeWrapper>;
            ps->readFromFile(ss.str());
            CSubmapNode* node = new CSubmapNode(ps);
            graph_mgr.addNode(node);
            ROS_INFO("add into %d submap Node!", i);
        }
        string precloud("preg2o.pcd");
        string poscloud("posg2o.pcd");
        string precloud2("preg2o2.pcd");
        string poscloud2("posg2o2.pcd");
        string featurecloud("feature.pcd");
        string g2of("g2o.log");
        graph_mgr.saveCloud2File(precloud);
        graph_mgr.saveCloudByOctomap(precloud2);
        graph_mgr.optimizeSubmapGraph();
        graph_mgr.saveG2OGraph(g2of.c_str());
        graph_mgr.saveFeature2File(featurecloud);
        graph_mgr.saveCloud2File(poscloud);
        graph_mgr.saveCloudByOctomap(poscloud2);
        ROS_ERROR("start to save refined submaps!");
        graph_mgr.saveRefinedSubmaps();
        ROS_INFO("finsih saving the whole graph!");
    }
    ROS_INFO("finished adding SubmapNode!");
    ROS_INFO("start to save into a whole one!");
    // string allcloud("allone.pcd");
    // graph_mgr.saveCloudsToFile(allone.c_str());
    // graph_mgr.saveCloud2File(allcloud);
    // ROS_INFO("finsih saving the whole graph!");

    //If one thread receives a exit signal from the user, signal the other thread to quit too
    QObject::connect(&app, SIGNAL(aboutToQuit()), &qtRos, SLOT(quitNow()));
    QObject::connect(&qtRos, SIGNAL(rosQuits()), &app, SLOT(quit()));

    // qtRos.start();// Run main loop.
    // app.exec();
    return 0;
}
