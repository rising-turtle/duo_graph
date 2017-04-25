/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

/*  Plane-based Map (PbMap) library
 *  Construction of plane-based maps and localization in it from RGBD Images.
 *  Writen by Eduardo Fernandez-Moral. See docs for <a href="group__mrpt__pbmap__grp.html" >mrpt-pbmap</a>
 */
#include <PbMap.h> // precomp. hdr
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <stdlib.h>
#include <stdio.h>

using namespace std;

/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
PbMap::PbMap() :
    FloorPlane(-1),
    globalMapPtr( new pcl::PointCloud<pcl::PointXYZRGBA>() ),
    edgeCloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA>),
    outEdgeCloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA>)
{
}

PbMap::~PbMap()
{
}
// Merge two pbmaps
void PbMap::MergeWith(PbMap &pbm, Eigen::Matrix4f &T)
{
  // Rotate and translate PbMap
  for(size_t i = 0; i < pbm.vPlanes.size(); i++)
  {
    Plane &plane = pbm.vPlanes[i];

    // Transform normal and ppal direction
    plane.v3normal = T.block(0,0,3,3) * plane.v3normal;
    plane.v3PpalDir = T.block(0,0,3,3) * plane.v3PpalDir;

    // Transform centroid
    plane.v3center = T.block(0,0,3,3) * plane.v3center + T.block(0,3,3,1);

    // Transform convex hull points
    pcl::transformPointCloud(*plane.polygonContourPtr, *plane.polygonContourPtr, T);

    vPlanes.push_back(plane);
  }

  // Rotate and translate the point cloud
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr alignedPointCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::transformPointCloud(*pbm.globalMapPtr,*alignedPointCloud,T);

  *globalMapPtr += *alignedPointCloud;

}

#include <fstream>
// Print PbMap content to a text file
void PbMap::printPbMap(string txtFilePbm)
{
cout << "PbMap 0.2\n\n";

  ofstream pbm;
  pbm.open(txtFilePbm.c_str());
  pbm << "PbMap 0.2\n\n";
  pbm << "MapPlanes " << vPlanes.size() << endl;
  for(unsigned i=0; i < vPlanes.size(); i++)
  {
    pbm << " ID " << vPlanes[i].id << " obs " << vPlanes[i].numObservations;
    pbm << " areaVoxels " << vPlanes[i].areaVoxels << " areaHull " << vPlanes[i].areaHull;
    pbm << " ratioXY " << vPlanes[i].elongation << " structure " << vPlanes[i].bFromStructure << " label " << vPlanes[i].label;
    pbm << "\n normal\n" << vPlanes[i].v3normal << "\n center\n" << vPlanes[i].v3center;
    pbm << "\n PpalComp\n" << vPlanes[i].v3PpalDir << "\n RGB\n" << vPlanes[i].v3colorNrgb;
    pbm << "\n Neighbors (" << vPlanes[i].neighborPlanes.size() << "): ";
    for(map<unsigned,unsigned>::iterator it=vPlanes[i].neighborPlanes.begin(); it != vPlanes[i].neighborPlanes.end(); it++)
      pbm << it->first << " ";
    pbm << "\n CommonObservations: ";
    for(map<unsigned,unsigned>::iterator it=vPlanes[i].neighborPlanes.begin(); it != vPlanes[i].neighborPlanes.end(); it++)
      pbm << it->second << " ";
    pbm << "\n ConvexHull (" << vPlanes[i].polygonContourPtr->size() << "): \n";
    for(unsigned j=0; j < vPlanes[i].polygonContourPtr->size(); j++)
      pbm << "\t" << vPlanes[i].polygonContourPtr->points[j].x << " " << vPlanes[i].polygonContourPtr->points[j].y << " " << vPlanes[i].polygonContourPtr->points[j].z << endl;
    pbm << endl;
  }
  pbm.close();
}

bool  PbMap::LoadPbMap(std::string txtFilePbm)
{
//	cout <<"File name: "<<txtFilePbm<<endl;
	ifstream pbm;
	pbm.open(txtFilePbm.c_str());

	string str_temp;
	double num_temp;
	pbm>>str_temp;  //PbMap
	if (str_temp != "PbMap")
	{
		return false;
	}

	pbm >> num_temp; //0.2
	pbm >> str_temp >> num_temp;
	int vplanesize = (int)num_temp;
//	cout <<"vplanesize = "<<vplanesize<<endl;
	if (str_temp!="MapPlanes")
	{
		return false;
	}
	vPlanes.clear();
	int idtemp, numObservationstemp;
	unsigned int untemp;
	float ftemp;
	float x,y,z;
	string strx,stry,strz;
//	pcl::PointXYZRGBA PtTemp;

	for(unsigned i=0; i < vplanesize; i++)
	{
		Plane nowPlane;
	    pbm >> str_temp >> idtemp;
	    nowPlane.id = idtemp;
	    pbm >> str_temp >> nowPlane.numObservations;
	    pbm >> str_temp >> nowPlane.areaVoxels ;
	    pbm >> str_temp >> nowPlane.areaHull;
	    pbm >> str_temp >> nowPlane.elongation;
//	    cout << "nowPlane.elongation = "<<nowPlane.elongation<<endl;
	    pbm >> str_temp >> nowPlane.bFromStructure >> str_temp;
	    nowPlane.label = "";
//	    cout << "Loading head information done."<<endl;

	    // V3normal
	    pbm >> str_temp >> strx >> stry >> strz;
	    nowPlane.v3normal(0) = atof(strx.c_str());
	    nowPlane.v3normal(1) = atof(stry.c_str());
	    nowPlane.v3normal(2) = atof(strz.c_str());
//	    cout << "Loading normal done."<<endl;
//	    cout << nowPlane.v3normal<<endl;

	    //V3Center
	    pbm >> str_temp >> strx >> stry >> strz;
	    nowPlane.v3center(0) = atof(strx.c_str());
	    nowPlane.v3center(1) = atof(stry.c_str());
	    nowPlane.v3center(2) = atof(strz.c_str());
//	    cout << "Loading center done."<<endl;
//	    cout << nowPlane.v3center<<endl;

	    //V3Pair
	    pbm >> str_temp >> strx >> stry >> strz;
	    nowPlane.v3PpalDir(0) = atof(strx.c_str());
	    nowPlane.v3PpalDir(1) = atof(stry.c_str());
	    nowPlane.v3PpalDir(2) = atof(strz.c_str());
//	    cout << "Loading v3Pair done."<<endl;
//	    cout << nowPlane.v3PpalDir<<endl;

	    //RGB
	    pbm >> str_temp >> strx >> stry >> strz;
	    nowPlane.v3colorNrgb(0) = atof(strx.c_str());
	    nowPlane.v3colorNrgb(1) = atof(stry.c_str());
	    nowPlane.v3colorNrgb(2) = atof(strz.c_str());
//	    cout << "Loading RGB done."<<endl;
//	    cout << nowPlane.v3colorNrgb<<endl;

	    // Neibors
	    int nNeighborNum;
	    pbm>>str_temp;
	    pbm>>str_temp;
		const char* chr_temp = str_temp.c_str();
		nNeighborNum = 0;
		for (int j=1; j<sizeof(chr_temp); j++)
		{
			if (chr_temp[j] == ')')
			{
				break;
			}
			nNeighborNum = nNeighborNum * 10 + (chr_temp[j] - '0');
		}
//		cout << "nNeighborNum = "<<nNeighborNum<<endl;
	    unsigned int neiborid[nNeighborNum];
	    unsigned int observationnum[nNeighborNum];
//	    cout <<"Neighbor ID: ";
		for (int j=0; j<nNeighborNum; j++)
		{
			pbm>>neiborid[j];
//			cout << neiborid[j]<< " ";
		}
//		cout <<endl;
		pbm>>str_temp;//CommonObservations
//	    cout <<"CommonObservations Numbers : ";
		for (int j=0; j<nNeighborNum; j++)
		{
			pbm>>observationnum[j];
//			cout << observationnum[j]<<" ";
		}
//		cout <<endl;

	    for(int k = 0; k< nNeighborNum; k++)
	    {
	    	nowPlane.neighborPlanes.insert(pair<unsigned, unsigned>(neiborid[k],observationnum[k]));
	    }
//	    cout << "Inserting neighbors done."<<endl;

		pbm>>str_temp;//convexHull
		pbm>>str_temp;
		chr_temp = str_temp.c_str();
		int converx_num = 0;
		for (int j=1; j<sizeof(chr_temp); j++)
		{
			if (chr_temp[j] == ')')
			{
				break;
			}
			converx_num = converx_num * 10 + (chr_temp[j] - '0');
		}
//		cout <<"converx_num = "<<converx_num <<endl;
		nowPlane.polygonContourPtr.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
		for (int j=0; j<converx_num; j++)
		{
	    	pbm >>x>>y>>z;
	    	pcl::PointXYZRGBA PtTemp;
	    	PtTemp.x = x;
	    	PtTemp.y = y;
	    	PtTemp.z = z;
	    	PtTemp.r = 1.0;
	    	PtTemp.g = 0.0;
	    	PtTemp.b = 0.0;
	    	nowPlane.polygonContourPtr->push_back(PtTemp);
		}
//		cout <<endl;
	    vPlanes.push_back(nowPlane);
	 }
	  pbm.close();

//	  cout <<"Loading file: "<<txtFilePbm<<endl;
//	  cout <<"Plane numbers: "<<vPlanes.size()<<endl;

	return true;
}
