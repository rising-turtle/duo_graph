
#include <iostream>     // std::cout
#include <fstream>      // std::ifstream
#include <string>
#include <vector>
#include <cmath>
#include <EDPbmap.h>

#include <PbMap.h>
#include <PbMapVisualizer.h>

using namespace std;

void printHelp()
{
    cout<<"---------------------------------------------------------------------------------------"<< endl;
    cout<<"./pbmap_visualizer <pointCloud.pcd> <planes.pbmap>" << endl;
    cout<<"       options: " << endl;
    cout<<"         -p | P: Show/hide point cloud" << endl;
    cout<<"         -l | L: Show/hide PbMap" << endl;
    cout<<"         -r | R: Switch between point cloud and graph representation" << endl;
}

int main(int argc, char** argv)
{

	  if(argc != 3)
	  {
		  printHelp();
		  return -1;
	  }

	  string pointCloudFile = static_cast<string>(argv[1]);
	  string PbMapFile = static_cast<string>(argv[2]);

	  PbMapVisualizer mapViewer;

	  // Read in the cloud data
	  pcl::PCDReader reader;
	  reader.read (pointCloudFile, *mapViewer.pbmap.globalMapPtr);
	  cout << "Size " << mapViewer.pbmap.globalMapPtr->size() << " " << mapViewer.pbmap.globalMapPtr->empty() << endl;

	  if (!mapViewer.pbmap.LoadPbMap(PbMapFile))
	  {
		  cout << "Error: cannot open " << PbMapFile << "\n";
		  return -1;
	  }
	  else

	  // Visualize PbMap
	  mapViewer.Visualize();

	  return 0;

}


