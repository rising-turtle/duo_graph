#include <EDPbmap.h>
#include <Plane.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <PbmapmakerPerFrame.h>
#include <ConsistencyTest.h>
#include <planeMatching.h>
#include <strstream>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/time.h>


using namespace std;

//string path("../data/");
string path("../data/pcdAndPose/");
//string path("/home/helen/Data/coffeeRoom/PCD-Pose/");
string despath("../data/PbMaps/");

void printHelp()
{
  cout << "./pbmap_test reads a set of pairs pointCloud+Pose to build a PbMap\n";
}

bool LoadPoseFile(string posefilename, Eigen::Matrix4f &inputpose)
{
    ifstream myfile (posefilename.c_str());
    cout<<"Current pose file :"<<posefilename<<endl;
    if(!myfile){
    	cout << "Unable to open posefile "<<posefilename<<endl;
    	return false; // terminate with error
    }
    int i=0;
    double fpose[16]={0};
    while (! myfile.eof() )
    {
    	myfile>>fpose[i];
    	i++;
    	if(i==16)
    		break;
    }
    cout<<endl;
    int l1,l2;
    for(l1=0;l1<4;l1++){
		for(l2 = 0; l2< 4; l2++)
		{
			inputpose(l1,l2) = (float)fpose[l1*4+l2];
		}
    }
    myfile.close();
    return true;
}

void GeneratePCDPose(string cloudfilename, string posefilename,frameRGBDandPose &inputcloudAndPose, Eigen::Matrix4f &eigenpose)
{
	  pcl::PCDReader reader;
      reader.read (path + cloudfilename, *inputcloudAndPose.cloudPtr);
      cout<<"Loading the pointcloud "<<cloudfilename<<endl;
      cout<<"The point cloud has the size of "<<inputcloudAndPose.cloudPtr->size()<<endl;
      LoadPoseFile(path+posefilename, eigenpose);
      // Read pose
	  for (int j = 0; j<16; j++)
		{
			inputcloudAndPose.pose[(int)(j/4)][j%4] = eigenpose((int)(j/4),j%4);
		}
	    int l1,l2;
	    for(l1=0;l1<4;l1++){
			for(l2 = 0; l2< 4; l2++)
			{
				cout<<inputcloudAndPose.pose[l1][l2] <<" ";
			}
			cout<<endl;
	    }
}
// input: 	pcd file name
//output:	Pbmap contains plane information.
void generatePbMap(const string cloudFile, const string PbMapFile, const string &config_file)
{
	  cout << config_file<<endl;
	  cout << "Cloud File: "<<cloudFile<<endl;

//	  cout<<"I will enter into the pbmaker.\n";
	  PbMapMaker pbmap_maker(config_file);
//	  cout << "Loading the config file done!\n";

      frameRGBDandPose cloudAndPose;
	  cloudAndPose.cloudPtr.reset(new pcl::PointCloud<PointT>);
	  for (int l1 = 0; l1 <4; l1 ++)
	  {
		  for(int l2 = 0; l2<4; l2++)
		  {
			  if(l1 == l2)
				  cloudAndPose.pose[l1][l2] = 1;
			  else
				  cloudAndPose.pose[l1][l2] = 0;
		  }
	  }

//	  cout <<"Point reset done."<<endl;
//	  cout<<"After reset, the point cloud has the size of "<<cloudAndPose.cloudPtr->size()<<endl;

	  pcl::PCDReader reader;
//	  pcl::PointCloud<PointT> cloudtemp;
//	  reader.read(cloudFile,cloudtemp);
//	  cout<<"Loading into cloud temp."<<endl;
	  reader.read (cloudFile, *cloudAndPose.cloudPtr);
	  cout<<"Loading the pointcloud "<<cloudFile<<endl;
	  cout<<"The point cloud has the size of "<<cloudAndPose.cloudPtr->size()<<endl;

	  pbmap_maker.frameQueue.push_back(cloudAndPose);
//	  cout << "pbmap maker push back done!"<<endl;
	  pbmap_maker.run();

//	  cout << "enter the serialize Pbmap!"<<endl;
//	  cout <<"Serialize Pbmap Name: "<<PbMapFile<<endl;

	  pbmap_maker.getPbMap().printPbMap(PbMapFile);

	  // Serialize PbMap
	  cout << "Serialize Pbmap Done!"<<endl;
	  // Save reconstructed point cloud

	  double total_area = 0.0;
	  for(unsigned i=0; i < pbmap_maker.getPbMap().vPlanes.size(); i++)
	    total_area += pbmap_maker.getPbMap().vPlanes[i].areaHull;
	  cout << "This PbMap contains " << pbmap_maker.getPbMap().vPlanes.size() << " planes, covering a total area of " << total_area << " m2" << endl;
}

void generateTotalPbMap(const string &config_file)
{
	  int idstart = 100;
	  int N = 1500;
	  string cloudFile, PbmapFile;
	  for(unsigned i=idstart; i < N+idstart; i++)
	  {
	    // Generate file name
	    string idstr;
	    strstream stemp;
	    ostringstream ost;
	    ost<<setfill('0');
	    ost<<setw(3)<<i;

	    idstr = ost.str();
	    string filenametemp = "pcd_000";
	    cloudFile = filenametemp + idstr;
	    filenametemp = ".pcd";
	    PbmapFile = despath + cloudFile +"-Pbmap.txt";
	    cloudFile = path + cloudFile +filenametemp; // "pointcloud%i.pcd"
	    generatePbMap(cloudFile, PbmapFile, config_file);
	  }
}

// bool PoseMode = true if it is the result calculating between two neighbor frames,
// bool PoseMode = false if the pose file is the final pose gotten from rgbm slam;
void MergePCDwithPlanes(vector<string> vstrpcdinput, vector<string> vstrposeinput, pcl::PointCloud<pcl::PointXYZRGBA> &globalCloud, bool PoseMode)
{
	if(vstrpcdinput.size()!=vstrposeinput.size())
	{
		cout<<"Input pcd file numbers doesn't equal to pose file number."<<endl;
		return;
	}
	Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f basepose = Eigen::Matrix4f::Identity();
	pcl::PointCloud<PointT>::Ptr globalCloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA>());
	for(unsigned i = 0; i< vstrpcdinput.size(); i++)
	{
		cout<<"pcd file: "<< vstrpcdinput[i]<<endl;
		cout<<"pose file: "<< vstrposeinput[i]<<endl;
		if(PoseMode)
		{
			if(vstrposeinput[i] == "")
				pose = Eigen::Matrix4f::Identity();
			else
				LoadPoseFile(vstrposeinput[i], pose);
			basepose = basepose*pose;
		}
		else
		{
			LoadPoseFile(vstrposeinput[i], basepose);
		}


		pcl::PointCloud<PointT>::Ptr nowCloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA>());
		cout<<"Now Pose: "<<endl;
		cout<<basepose<<endl;

		pcl::PCDReader reader;
	    reader.read (vstrpcdinput[i], *nowCloudPtr);

	    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr alignedCloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA>);
	    pcl::transformPointCloud(*nowCloudPtr,*alignedCloudPtr,basepose);
	    *globalCloudPtr += *alignedCloudPtr;

	    static pcl::VoxelGrid<pcl::PointXYZRGBA> grid;
	    grid.setLeafSize(0.01,0.01,0.01);
	    //grid.setLeafSize(0.05,0.05,0.05);
	    grid.setInputCloud (globalCloudPtr);
	    grid.filter (globalCloud);
	    globalCloudPtr->clear();
	    *globalCloudPtr = globalCloud;
	    //cloudAndPose.cloudPtr->clear();
	}
    cout << "global could update done!"<<endl;
    if(PoseMode)
    	pcl::io::savePCDFile("../data/reconstructed_cloud.pcd", globalCloud);
    else
    	pcl::io::savePCDFile("../data/reconstructed_cloud_slam.pcd", globalCloud);

    cout << "Saving PCD File Done!"<<endl;
}

void PoseEstimation()
{
//	int idstart = 1;
//	int idend = 1039;
	int idstart = 448;
	int idend = 467;
	PbMap PrePbmap;
	PbMap NowPbmap;
	char txt_front[1024];
	char txt_back[1024];
	char txt_poseres[1024];
	char txt_slampose[1024];
	Eigen::Matrix4f pose;
	char c;
	int EnoughMatchPairNum = 0;
	int nContinuousPair = 0;
	int nNowLongest = 0;
	int nGeneralLongest = -1;
	int nStartId = idstart;
	int nNowStartId = idstart;
	bool isPreInMatchList = false;
	vector<string> vstrpcdinput;
	vector<string> vstrposeoutput;
	vector<string> vstrslampose;
	ofstream match_out_f("../data/MatchRes.txt");
	for(unsigned i = idstart; i< idend; i++)
	{
		cout<<"Matching the pair: "<<i<<endl;
//		sprintf(txt_front, "/home/helen/workspace/PoseEstimation/data/PbMaps/pcd_%.6d-Pbmap.txt", i);
//		sprintf(txt_back, "/home/helen/workspace/PoseEstimation/data/PbMaps/pcd_%.6d-Pbmap.txt", i+1);
		sprintf(txt_front, "../data/PbMaps/pcd_%.6d-Pbmap.txt", i);
		sprintf(txt_back,  "../data/PbMaps/pcd_%.6d-Pbmap.txt", i+1);
		PrePbmap.LoadPbMap(txt_front);
		NowPbmap.LoadPbMap(txt_back);
		PlaneMatching m_planematching(PrePbmap, NowPbmap);
		cout<<"Matched planes number: "<<m_planematching.matched_planes.size()<<endl;
		if(m_planematching.matched_planes.size()<=3)
		{
			cout<<"Matching failed. Doesn't have enough planes."<<endl;
			isPreInMatchList = false;
			if(nNowLongest>nGeneralLongest)
			{
				nGeneralLongest = nNowLongest>0?nNowLongest:0;
				nStartId = nNowStartId;
			}
			continue;
		}
		sprintf(txt_poseres, "../data/PoseResult/pose_%.6d.txt", i+1);
		ConsistencyTest m_consistencytest(PrePbmap, NowPbmap);
		pose = m_consistencytest.getRTwithModelLY(m_planematching.matched_planes);
		pose = inverse(pose);
		ofstream out_f(txt_poseres);
		for(int l1 = 0; l1< 4; l1++)
		{
			for(int l2 = 0; l2< 4; l2++)
			{
				out_f<<pose(l1,l2)<<" ";
			}
		}
		out_f.close();
		cout <<"Now pose: "<<endl;
		cout << pose<<endl;
		EnoughMatchPairNum++;
		if(!isPreInMatchList)
		{
			match_out_f<<nNowStartId<<"\t"<<nNowLongest<<endl;
			nNowStartId = i;
			nNowLongest = 1;
			isPreInMatchList = true;
		}
		else
		{
			nNowLongest++;
			cout<<"nNowLongest = "<<nNowLongest<<endl;
			isPreInMatchList = true;
		}
	}
	match_out_f.close();
	cout<<"EnoughMatchPairNum = "<<EnoughMatchPairNum<<endl;
	cout<<"Longest match has "<< nGeneralLongest<<" pair of matched frames."<<endl;
	cout<<"The matching start from: "<<nStartId<<endl;

	nStartId = 448;
	nGeneralLongest = 20;
	sprintf(txt_front, "../data/pcdAndPose/pcd_%.6d.pcd", nStartId);
	sprintf(txt_slampose, "../data/pcdAndPose/pose_%.6d.txt", nStartId);
	vstrpcdinput.push_back(txt_front);
	vstrposeoutput.push_back("");
	vstrslampose.push_back(txt_slampose);
	for(int l1 = 1; l1 <= nGeneralLongest; l1++)
	{
		sprintf(txt_front, "../data/pcdAndPose/pcd_%.6d.pcd", nStartId+l1);
		sprintf(txt_slampose, "../data/pcdAndPose/pose_%.6d.txt", nStartId+l1);
		sprintf(txt_poseres, "../data/PoseResult/pose_%.6d.txt", nStartId+l1);

		vstrpcdinput.push_back(txt_front);
		vstrposeoutput.push_back(txt_poseres);
		vstrslampose.push_back(txt_slampose);
	}
	pcl::PointCloud<pcl::PointXYZRGBA> res_cloud;
	MergePCDwithPlanes(vstrpcdinput, vstrposeoutput, res_cloud, true);

	pcl::PointCloud<pcl::PointXYZRGBA> slam_res_cloud;
	MergePCDwithPlanes(vstrpcdinput,vstrslampose, slam_res_cloud, false);

}


int main(int argc, char **argv)
{
 try
 {
	bool showHelp = false;

	// Process arguments:
	if (argc<2 )
	{
		printf("Usage: %s <config_file.ini>\n\n",argv[0]);
		if (!showHelp)
		{
			return -1;
		}
		else	return 0;
	}

   const string INI_FILENAME = string( argv[1] );

   //generateTotalPbMap(INI_FILENAME);

   PoseEstimation();

   return 0;

 } catch (exception &e)
 {
   cout << "MRPT exception caught: " << e.what() << endl;
   return -1;
 }
 catch (...)
 {
   printf("Another exception!!");
   return -1;
 }
}
