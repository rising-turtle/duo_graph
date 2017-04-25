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
	QtROS qtRos(argc, argv, "mainMapBuilderOffline");
	//Depending an use_gui on the Parameter Server, a gui- or a headless application is used
	QApplication app(argc, argv, ParameterServer::instance()->get<bool>("use_gui"));
	string path = "./submaps";

	int i;
	if(argc >= 2)
	{
		/*
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
		 */
		int start_n = atoi(argv[1]);
		int end_n = atoi(argv[2]);
		ROS_INFO("Input number is %d - %d", start_n, end_n);
		CSubmapGraph graph_mgr;
		ParameterServer * ps = ParameterServer::instance();
		ps->set<int>("ransac_iterations" , 200);
		ps->set<double>("max_dist_for_inliers", 5);

		for(int i=start_n; i<end_n; i++)
		{
			stringstream ss;
			ss<<path<<"/"<<i;
			//ss<<setfill('0')<<setw(2)<<i;
			ss<<"submap.map";
			ROS_ERROR(ss.str().c_str());

			CSubmap<CNodeWrapper> * ps = new CSubmap<CNodeWrapper>;
			ps->readFromFile(ss.str());
			CSubmapNode* node = new CSubmapNode(ps);
			graph_mgr.addNode(node);
			ROS_INFO("add into %d submap Node!", i);
		}
		string precloud("preg2o.pcd");
		string poscloud("posg2o.pcd");
		string precloud2("preg2o.ot");
		string poscloud2("posg2o.ot");
		string featurecloud("feature.pcd");
		string g2of("g2o.log");
		string featureMap("preglobalFeature.map");
		string featureMap2("posglobalFeature.map");

		graph_mgr.saveCloud2File(precloud);
		graph_mgr.saveCloudByOctomap(precloud2);
		graph_mgr.saveFeaturePosDes2File(featureMap);
		graph_mgr.optimizeSubmapGraph();
		graph_mgr.saveFeaturePosDes2File(featureMap2);
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
