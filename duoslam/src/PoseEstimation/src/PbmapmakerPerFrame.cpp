
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

#include <PbmapmakerPerFrame.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/time.h>
#include <iostream>

//#define _VERBOSE 1

using namespace std;
using namespace Eigen;


#define _VERBOSE 1


/*!Some parameters to specify input/output and some thresholds.*/
struct config_pbmap
{
  // [global]
  bool input_from_rawlog;
  std::string rawlog_path;
  bool record_rawlog;
  float color_threshold;

  // [plane_segmentation]
  float dist_threshold; // Maximum distance to the plane between neighbor 3D-points
  float angle_threshold; //  = 0.017453 * 4.0 // Maximum angle between contiguous 3D-points
  float minInliersRate; // Minimum ratio of inliers/image points required

  // [map_construction]
  bool use_color;                   // Add color information to the planes
  float proximity_neighbor_planes;  // Two planar patches are considered neighbors when the closest distance between them is under proximity_neighbor_planes
//  float max_angle_normals; // (10º) Two planar patches that represent the same surface must have similar normals // QUITAR
  float max_cos_normal;
  float max_dist_center_plane; // Two planar patches that represent the same surface must have their center in the same plane
  float proximity_threshold;  // Two planar patches that represent the same surface must overlap or be nearby
  int   graph_mode;  // This var selects the condition to create edges in the graph, either proximity of planar patches or co-visibility in a single frame

  // [semantics]
  bool inferStructure;    // Infer if the planes correspond to the floor, ceiling or walls
  bool makeClusters; // Should the PbMapMaker cluster the planes according to their co-visibility

  // [localisation]
  bool detect_loopClosure;             // Run PbMapLocaliser in a different threads to detect loop closures or preloaded PbMaps
  string config_localiser;

  // [serialize]
  std::string path_save_pbmap;
  bool save_registered_cloud;
  std::string path_save_registered_cloud;

} configPbMap;

void readConfigFile(const string &config_file_name)
{
 /* mrpt::utils::CConfigFile config_file(config_file_name);

  // Plane segmentation
  configPbMap.dist_threshold = config_file.read_float("plane_segmentation","dist_threshold",0.04,true);
  configPbMap.angle_threshold = config_file.read_float("plane_segmentation","angle_threshold",0.069812,true);
  configPbMap.minInliersRate = config_file.read_float("plane_segmentation","minInliersRate",0.01,true);

  // map_construction
  configPbMap.use_color = config_file.read_bool("map_construction","use_color",false);
  configPbMap.proximity_neighbor_planes = config_file.read_float("map_construction","proximity_neighbor_planes",1.0);
  configPbMap.graph_mode = config_file.read_int("map_construction","graph_mode",0);
  configPbMap.max_cos_normal = config_file.read_float("map_construction","max_cos_normal",0.9848,true);
  configPbMap.max_dist_center_plane = config_file.read_float("map_construction","max_dist_center_plane",0.1,true);
  configPbMap.proximity_threshold = config_file.read_float("map_construction","proximity_threshold",0.15);

  // [semantics]
  configPbMap.inferStructure = config_file.read_bool("semantics","inferStructure",true);
  configPbMap.makeClusters = config_file.read_bool("semantics","makeClusters",true);
//  configPbMap.path_prev_pbmap = config_file.read_string("localisation","path_prev_pbmap","",true);

  // [localisation]
  configPbMap.detect_loopClosure = config_file.read_bool("localisation","detect_loopClosure",true);
  if(configPbMap.detect_loopClosure)
    configPbMap.config_localiser = config_file.read_string("localisation","config_localiser","",true);

  // serialize
  configPbMap.path_save_pbmap = config_file.read_string("serialize","path_save_pbmap","map");
  configPbMap.save_registered_cloud = config_file.read_bool("serialize","save_registered_cloud",true);
  configPbMap.path_save_registered_cloud = config_file.read_string("serialize","path_save_registered_cloud","/home/helen/workspace/MRPT/Projects/PbMaps/PbMaps.txt");
//cout << "path_save_registered_cloud " << configPbMap.path_save_registered_cloud << endl;

  #ifdef _VERBOSE
    cout << "readConfigFile configPbMap.ini dist_threshold " << configPbMap.dist_threshold << endl;
  #endif
  */
	  configPbMap.dist_threshold = 0.08; //0.03
	  configPbMap.angle_threshold = 0.087265; //0.069812 = 0.017453 * 4.0
	  configPbMap.minInliersRate = 0.0005;	//0.005 0.001

//	  configPbMap.dist_threshold = 0.02; //0.03
//	  configPbMap.angle_threshold = 0.017453 * 2.0; //0.069812 = 0.017453 * 4.0
//	  configPbMap.minInliersRate = 0.003;	//0.005 0.001

	  // map_construction
	  configPbMap.use_color = true; //false
	  configPbMap.proximity_neighbor_planes = 1.5;
	  configPbMap.graph_mode = 0;
	  configPbMap.max_cos_normal = 0.9848;
	  configPbMap.max_dist_center_plane = 0.12;
	  configPbMap.proximity_threshold = 0.15;
}

PbMapMaker::PbMapMaker(const string &config_file) :
    m_pbmaker_must_stop(false),
    m_pbmaker_finished(false)
{
//  GVars3::GUI.RegisterCommand("SavePbMap", GUICommandCallBack, this);

  // Load parameters
  readConfigFile(config_file);
//
//  if(pthread_create(&pbmaker_hd,NULL,ThreadPbMapMaker,this)!=0)
//  {
//  	printf("Create Thread Failed!!!!\n");
//  }

}

void*  PbMapMaker::ThreadPbMapMaker(void* lpParam)
{
	pthread_detach(pthread_self());
	PbMapMaker *pPbMapMaker=(PbMapMaker *)lpParam;
	pPbMapMaker->run();
	return 0;
}

bool PbMapMaker::arePlanesNearby(Plane &plane1, Plane &plane2, const float distThreshold)
{
  float distThres2 = distThreshold * distThreshold;

  // First we check distances between centroids and vertex to accelerate this check
  if( (plane1.v3center - plane2.v3center).squaredNorm() < distThres2 )
    return true;

  for(unsigned i=1; i < plane1.polygonContourPtr->size(); i++)
    if( (getVector3fromPointXYZ(plane1.polygonContourPtr->points[i]) - plane2.v3center).squaredNorm() < distThres2 )
      return true;

  for(unsigned j=1; j < plane2.polygonContourPtr->size(); j++)
    if( (plane1.v3center - getVector3fromPointXYZ(plane2.polygonContourPtr->points[j]) ).squaredNorm() < distThres2 )
      return true;

  for(unsigned i=1; i < plane1.polygonContourPtr->size(); i++)
    for(unsigned j=1; j < plane2.polygonContourPtr->size(); j++)
      if( (diffPoints(plane1.polygonContourPtr->points[i], plane2.polygonContourPtr->points[j]) ).squaredNorm() < distThres2 )
        return true;

  //If not found yet, search properly by checking distances:
  // a) Between an edge and a vertex
  // b) Between two edges (imagine two polygons on perpendicular planes)
  // c) Between a vertex and the inside of the polygon
  // d) Or the polygons intersect

  // a) & b)
  for(unsigned i=1; i < plane1.polygonContourPtr->size(); i++)
    for(unsigned j=1; j < plane2.polygonContourPtr->size(); j++)
      if(dist3D_Segment_to_Segment2(Segment(plane1.polygonContourPtr->points[i],plane1.polygonContourPtr->points[i-1]), Segment(plane2.polygonContourPtr->points[j],plane2.polygonContourPtr->points[j-1])) < distThres2)
        return true;

  // c)
  for(unsigned i=1; i < plane1.polygonContourPtr->size(); i++)
    if( plane2.v3normal.dot(getVector3fromPointXYZ(plane1.polygonContourPtr->points[i]) - plane2.v3center) < distThreshold )
      if(isInHull(plane1.polygonContourPtr->points[i], plane2.polygonContourPtr) )
        return true;

  for(unsigned j=1; j < plane2.polygonContourPtr->size(); j++)
    if( plane1.v3normal.dot(getVector3fromPointXYZ(plane2.polygonContourPtr->points[j]) - plane1.v3center) < distThreshold )
      if(isInHull(plane2.polygonContourPtr->points[j], plane1.polygonContourPtr) )
        return true;

  return false;
}

void PbMapMaker::checkProximity(Plane &plane, float proximity)
{
  for(unsigned i=0; i < mPbMap.vPlanes.size(); i++ )
  {
    if(plane.id == mPbMap.vPlanes[i].id)
      continue;

    if(plane.nearbyPlanes.count(mPbMap.vPlanes[i].id))
      continue;

    if(arePlanesNearby(plane, mPbMap.vPlanes[i], proximity) ) // If the planes are closer than proximity (in meters), then mark them as neighbors
    {
//      cout<<"plane "<<mPbMap.vPlanes[i].id<<"is my neighbor."<<endl;
      plane.nearbyPlanes.insert(mPbMap.vPlanes[i].id);
      mPbMap.vPlanes[i].nearbyPlanes.insert(plane.id);
    }
  }
}


/*void PbMapMaker::detectPlanesCloud( pcl::PointCloud<PointT>::Ptr &pointCloudPtr_arg,
                                    Eigen::Matrix4f &poseKF,
                                    double distThreshold, double angleThreshold, double minInliersF)*/
void PbMapMaker::detectPlanesCloud( pcl::PointCloud<PointT>::Ptr &pointCloudPtr_arg,
                                    float inputpose[4][4],
                                    double distThreshold, double angleThreshold, double minInliersF)
{
  unsigned minInliers = minInliersF * pointCloudPtr_arg->size();
  cout << "minInliers ="<<minInliers<<endl;
  #ifdef _VERBOSE
    cout << "detectPlanes in a cloud with " << pointCloudPtr_arg->size() << " points " << minInliers << " minInliers\n";
  #endif
  Eigen::Matrix4f poseKF;
  int l1,l2;
  for(l1=0;l1<4;l1++){
	for(l2 = 0; l2< 4; l2++)
	{
		poseKF(l1,l2) = inputpose[l1][l2];
	}
  }
  
  pcl::PointCloud<PointT>::Ptr pointCloudPtr_arg2(new pcl::PointCloud<PointT>);
  pcl::copyPointCloud(*pointCloudPtr_arg,*pointCloudPtr_arg2);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr alignedCloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::transformPointCloud(*pointCloudPtr_arg,*alignedCloudPtr,poseKF);

  { *mPbMap.globalMapPtr += *alignedCloudPtr;
  } // End CS

  // Downsample voxel map's point cloud
  static pcl::VoxelGrid<pcl::PointXYZRGBA> grid;
  grid.setLeafSize(0.02,0.02,0.02);
  pcl::PointCloud<pcl::PointXYZRGBA> globalMap;
  grid.setInputCloud (mPbMap.globalMapPtr);
  grid.filter (globalMap);
  mPbMap.globalMapPtr->clear();
  *mPbMap.globalMapPtr = globalMap;

  pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
  ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
  ne.setMaxDepthChangeFactor (0.02f); // For VGA: 0.02f, 10.0f
  ne.setNormalSmoothingSize (5.0f);
  ne.setDepthDependentSmoothing (true);

  pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
  mps.setMinInliers (minInliers);
  mps.setAngularThreshold (angleThreshold); // (0.017453 * 2.0) // 3 degrees
  mps.setDistanceThreshold (distThreshold); //2cm

  pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
  ne.setInputCloud (pointCloudPtr_arg2);
  ne.compute (*normal_cloud);

#ifdef _VERBOSE
  double plane_extract_start = pcl::getTime ();
#endif
  mps.setInputNormals (normal_cloud);
  mps.setInputCloud (pointCloudPtr_arg2);

  std::vector<pcl::PlanarRegion<PointT>, aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
  std::vector<pcl::ModelCoefficients> model_coefficients;
  std::vector<pcl::PointIndices> inlier_indices;
  pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
  std::vector<pcl::PointIndices> label_indices;
  std::vector<pcl::PointIndices> boundary_indices;
  mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);

  #ifdef _VERBOSE
    double plane_extract_end = pcl::getTime();
    std::cout << "Plane extraction took " << double (plane_extract_end - plane_extract_start) << std::endl;
//    std::cout << "Frame took " << double (plane_extract_end - normal_start) << std::endl;
    cout << regions.size() << " planes detected\n";
  #endif

  // Create a vector with the planes detected in this keyframe, and calculate their parameters (normal, center, pointclouds, etc.)
  // in the global reference
  vector<Plane> detectedPlanes;
  for (size_t i = 0; i < regions.size (); i++)
  {
    Plane plane;

    Vector3f centroid = regions[i].getCentroid ();
    plane.v3center = compose(poseKF, centroid);
    plane.v3normal = poseKF.block(0,0,3,3) * Vector3f(model_coefficients[i].values[0], model_coefficients[i].values[1], model_coefficients[i].values[2]);
//  assert(plane.v3normal*plane.v3center.transpose() <= 0);
//    if(plane.v3normal*plane.v3center.transpose() <= 0)
//      plane.v3normal *= -1;

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
    extract.setInputCloud (pointCloudPtr_arg2);
    extract.setIndices ( boost::make_shared<const pcl::PointIndices> (inlier_indices[i]) );
    extract.setNegative (false);
    extract.filter (*plane.planePointCloudPtr);    // Write the planar point cloud

    static pcl::VoxelGrid<pcl::PointXYZRGBA> plane_grid;
    plane_grid.setLeafSize(0.05,0.05,0.05);
    pcl::PointCloud<pcl::PointXYZRGBA> planeCloud;
    plane_grid.setInputCloud (plane.planePointCloudPtr);
    plane_grid.filter (planeCloud);
    plane.planePointCloudPtr->clear();
    pcl::transformPointCloud(planeCloud,*plane.planePointCloudPtr,poseKF);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr contourPtr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    contourPtr->points = regions[i].getContour();
//    plane.contourPtr->points = regions[i].getContour();
//    pcl::transformPointCloud(*plane.contourPtr,*plane.polygonContourPtr,poseKF);
    pcl::transformPointCloud(*contourPtr,*plane.polygonContourPtr,poseKF);
    plane_grid.setInputCloud (plane.polygonContourPtr);
//    plane_grid.filter (*plane.contourPtr);
//    plane.calcConvexHull(plane.contourPtr);
    plane_grid.filter (*contourPtr);
    plane.calcConvexHull(contourPtr);
    plane.computeMassCenterAndArea();
    plane.areaVoxels= plane.planePointCloudPtr->size() * 0.0025;

    #ifdef _VERBOSE
      cout << "Area plane region " << plane.areaVoxels<< " of Chull " << plane.areaHull << " of polygon " << plane.compute2DPolygonalArea() << endl;
    #endif

    // Check whether this region correspond to the same plane as a previous one (this situation may happen when there exists a small discontinuity in the observation)
    bool isSamePlane = false;
    for (size_t j = 0; j < detectedPlanes.size(); j++)
      if( areSamePlane(detectedPlanes[j], plane, configPbMap.max_cos_normal, configPbMap.max_dist_center_plane, configPbMap.proximity_threshold) ) // The planes are merged if they are the same
      {
        isSamePlane = true;

        mergePlanes(detectedPlanes[j], plane);

        #ifdef _VERBOSE
          cout << "\tTwo regions support the same plane in the same KeyFrame\n";
        #endif

        break;
      }
    if(!isSamePlane)
      detectedPlanes.push_back(plane);
  }

  #ifdef _VERBOSE
    cout << detectedPlanes.size () << " Planes detected\n";
  #endif

  // Merge detected planes with previous ones if they are the same
  size_t numPrevPlanes = mPbMap.vPlanes.size();
//  set<unsigned> observedPlanes;
  observedPlanes.clear();
  for (size_t i = 0; i < detectedPlanes.size (); i++)
  {
    // Check similarity with previous planes detected
    bool isSamePlane = false;
    vector<Plane>::iterator itPlane = mPbMap.vPlanes.begin();
//  if(frameQueue.size() != 12)
    for(size_t j = 0; j < numPrevPlanes; j++, itPlane++) // numPrevPlanes
    {
      if( areSamePlane(mPbMap.vPlanes[j], detectedPlanes[i], configPbMap.max_cos_normal, configPbMap.max_dist_center_plane, configPbMap.proximity_threshold) ) // The planes are merged if they are the same
      {

        isSamePlane = true;

        mergePlanes(mPbMap.vPlanes[j], detectedPlanes[i]);

        // Update proximity graph
        checkProximity(mPbMap.vPlanes[j], configPbMap.proximity_neighbor_planes); // Detect neighbors

        #ifdef _VERBOSE
          cout << "Previous plane " << mPbMap.vPlanes[j].id << " area " << mPbMap.vPlanes[j].areaVoxels<< " of polygon " << mPbMap.vPlanes[j].compute2DPolygonalArea() << endl;
        #endif

        if( observedPlanes.count(mPbMap.vPlanes[j].id) == 0 ) // If this plane has already been observed through a previous partial plane in this same keyframe, then we must not account twice in the observation count
        {
          mPbMap.vPlanes[j].numObservations++;

          // Update co-visibility graph
          for(set<unsigned>::iterator it = observedPlanes.begin(); it != observedPlanes.end(); it++)
            if(mPbMap.vPlanes[j].neighborPlanes.count(*it))
            {
              mPbMap.vPlanes[j].neighborPlanes[*it]++;
              mPbMap.vPlanes[*it].neighborPlanes[mPbMap.vPlanes[j].id]++; // j = mPbMap.vPlanes[j]
            }
            else
            {
              mPbMap.vPlanes[j].neighborPlanes[*it] = 1;
              mPbMap.vPlanes[*it].neighborPlanes[mPbMap.vPlanes[j].id] = 1;
            }

          observedPlanes.insert(mPbMap.vPlanes[j].id);
        }

        #ifdef _VERBOSE
          cout << "Same plane\n";
        #endif

        itPlane++;
        for(size_t k = j+1; k < numPrevPlanes; k++, itPlane++) // numPrevPlanes
          if( areSamePlane(mPbMap.vPlanes[j], mPbMap.vPlanes[k], configPbMap.max_cos_normal, configPbMap.max_dist_center_plane, configPbMap.proximity_threshold) ) // The planes are merged if they are the same
          {
            mergePlanes(mPbMap.vPlanes[j], mPbMap.vPlanes[k]);

            mPbMap.vPlanes[j].numObservations += mPbMap.vPlanes[k].numObservations;

            for(set<unsigned>::iterator it = mPbMap.vPlanes[k].nearbyPlanes.begin(); it != mPbMap.vPlanes[k].nearbyPlanes.end(); it++)
              mPbMap.vPlanes[*it].nearbyPlanes.erase(mPbMap.vPlanes[k].id);

            for(map<unsigned,unsigned>::iterator it = mPbMap.vPlanes[k].neighborPlanes.begin(); it != mPbMap.vPlanes[k].neighborPlanes.end(); it++)
              mPbMap.vPlanes[it->first].neighborPlanes.erase(mPbMap.vPlanes[k].id);

            // Update plane index
            for(size_t h = k+1; h < numPrevPlanes; h++)
              --mPbMap.vPlanes[h].id;

            for(size_t h = 0; h < numPrevPlanes; h++)
            {
              if(k==h)
                continue;

              for(set<unsigned>::iterator it = mPbMap.vPlanes[h].nearbyPlanes.begin(); it != mPbMap.vPlanes[h].nearbyPlanes.end(); it++)
                if(*it > mPbMap.vPlanes[k].id)
                {
                  mPbMap.vPlanes[h].nearbyPlanes.insert(*it-1);
                  mPbMap.vPlanes[h].nearbyPlanes.erase(*it);
                }

              for(map<unsigned,unsigned>::iterator it = mPbMap.vPlanes[h].neighborPlanes.begin(); it != mPbMap.vPlanes[h].neighborPlanes.end(); it++)
                if(it->first > mPbMap.vPlanes[k].id)
                {
                  mPbMap.vPlanes[h].neighborPlanes[it->first-1] = it->second;
                  mPbMap.vPlanes[h].neighborPlanes.erase(it);
                }
            }

            mPbMap.vPlanes.erase(itPlane);
            --numPrevPlanes;

            #ifdef _VERBOSE
              cout << "MERGE TWO PREVIOUS PLANES WHEREBY THE INCORPORATION OF A NEW REGION \n";
            #endif
          }

        break;
      }
    }
    if(!isSamePlane)
    {
      detectedPlanes[i].id = mPbMap.vPlanes.size();
      detectedPlanes[i].numObservations = 1;
      detectedPlanes[i].bFullExtent = false;
      detectedPlanes[i].nFramesAreaIsStable = 0;


      #ifdef _VERBOSE
        cout << "New plane " << detectedPlanes[i].id << " area " << detectedPlanes[i].areaVoxels<< " of polygon " << detectedPlanes[i].areaHull << endl;
      #endif

      // Update proximity graph
      checkProximity(detectedPlanes[i], configPbMap.proximity_neighbor_planes);  // Detect neighbors with max separation of 1.0 meters

      // Update co-visibility graph
      for(set<unsigned>::iterator it = observedPlanes.begin(); it != observedPlanes.end(); it++)
      {
        detectedPlanes[i].neighborPlanes[*it] = 1;
        mPbMap.vPlanes[*it].neighborPlanes[detectedPlanes[i].id] = 1;
      }

      observedPlanes.insert(detectedPlanes[i].id);

      mPbMap.vPlanes.push_back(detectedPlanes[i]);
    }
  }

 // if(frameQueue.size() == 12)
   //cout << "Same plane? " << areSamePlane(mPbMap.vPlanes[2], mPbMap.vPlanes[9], configPbMap.max_cos_normal, configPbMap.max_dist_center_plane, configPbMap.proximity_threshold) << endl;

  #ifdef _VERBOSE
    cout << "\n\tobservedPlanes: ";
    cout << observedPlanes.size () << " Planes observed\n";
    for(set<unsigned>::iterator it = observedPlanes.begin(); it != observedPlanes.end(); it++)
      cout << *it << " ";
    cout << endl;
  #endif

    // For all observed planes
    for(set<unsigned>::iterator it = observedPlanes.begin(); it != observedPlanes.end(); it++)
    {
      Plane &observedPlane = mPbMap.vPlanes[*it];
      //cout<<"Get plane "<<observedPlane.id<<" with number of "<<observedPlane.planePointCloudPtr->points.size ()<<" points"<<endl;
      // Calculate principal direction
      observedPlane.calcElongationAndPpalDir();
      //cout << "Plane " << observedPlane.id << " ElongationAndPpal Calculation done.\n";
      // Update color
      observedPlane.calcMainColor();
      //cout << "Plane " << observedPlane.id << " color\n" << observedPlane.v3colorNrgb << endl;

    } // End for obsevedPlanes

    cout<<"Observed planes calculation done."<<endl;

    // Search the floor plane
    if(mPbMap.FloorPlane != -1) // Verify that the observed planes centers are above the floor
    {
      #ifdef _VERBOSE
        cout << "Verify that the observed planes centers are above the floor\n";
      #endif

      for(set<unsigned>::reverse_iterator it = observedPlanes.rbegin(); it != observedPlanes.rend(); it++)
      {
        if(static_cast<int>(*it) == mPbMap.FloorPlane)
          continue;
        if( mPbMap.vPlanes[mPbMap.FloorPlane].v3normal.dot(mPbMap.vPlanes[*it].v3center - mPbMap.vPlanes[mPbMap.FloorPlane].v3center) < -0.1 )
        {
          if(mPbMap.vPlanes[mPbMap.FloorPlane].v3normal.dot(mPbMap.vPlanes[*it].v3normal) > 0.99) //(cos 8.1º = 0.99)
          {
            mPbMap.vPlanes[*it].label = "Floor";
            mPbMap.vPlanes[mPbMap.FloorPlane].label = "";
            mPbMap.FloorPlane = *it;
          }
          else
          {
//            assert(false);
            mPbMap.vPlanes[mPbMap.FloorPlane].label = "";
            mPbMap.FloorPlane = -1;
            break;
          }
        }
      }
    }

  for(l1=0;l1<4;l1++){
	for(l2 = 0; l2< 4; l2++)
	{
		inputpose[l1][l2] = poseKF(l1,l2);
	}
  }

    #ifdef _VERBOSE
      cout << "DetectedPlanesCloud finished\n";
    #endif
}


/*!Check if the the input plane is the same than this plane for some given angle and distance thresholds.
 * If the planes are the same they are merged in this and the function returns true. Otherwise it returns false.*/
bool PbMapMaker::areSamePlane(Plane &plane1, Plane &plane2, const float &cosAngleThreshold, const float &distThreshold, const float &proxThreshold)
{
  // Check that both planes have similar orientation
  if( plane1.v3normal.dot(plane2.v3normal) < cosAngleThreshold )
    return false;
//  if(plane1.id == 2)
//    cout << "normal " << plane1.v3normal.dot(plane2.v3normal) << " " << cosAngleThreshold << endl;

  // Check the normal distance of the planes centers using their average normal
  float dist_normal = plane1.v3normal.dot(plane2.v3center - plane1.v3center);
//  if(fabs(dist_normal) > distThreshold ) // Avoid matching different parallel planes
//    return false;
  float thres_max_dist = max(distThreshold, distThreshold*2*((plane2.v3center - plane1.v3center).norm()));
  if(fabs(dist_normal) > thres_max_dist ) // Avoid matching different parallel planes
    return false;
//  if(plane1.id == 2)
//  {
//    cout << "dist_normal " << dist_normal << " " << thres_max_dist << endl;
//    if(arePlanesNearby(plane1, plane2, proxThreshold))
//      cout << "planes rearby" << endl;
//  }

  // Once we know that the planes are almost coincident (parallelism and position)
  // we check that the distance between the planes is not too big
  return arePlanesNearby(plane1, plane2, proxThreshold);
}

void PbMapMaker::mergePlanes(Plane &updatePlane, Plane &discardPlane)
{
  // Update normal and center
  updatePlane.v3normal = updatePlane.areaVoxels*updatePlane.v3normal + discardPlane.areaVoxels*discardPlane.v3normal;
  updatePlane.v3normal = updatePlane.v3normal / updatePlane.v3normal.norm();
  // Update point inliers
//  *updatePlane.polygonContourPtr += *discardPlane.polygonContourPtr; // Merge polygon points
  *updatePlane.planePointCloudPtr += *discardPlane.planePointCloudPtr; // Add the points of the new detection and perform a voxel grid

  // Filter the points of the patch with a voxel-grid. This points are used only for visualization
  static pcl::VoxelGrid<pcl::PointXYZRGBA> merge_grid;
  merge_grid.setLeafSize(0.05,0.05,0.05);
  pcl::PointCloud<pcl::PointXYZRGBA> mergeCloud;
  merge_grid.setInputCloud (updatePlane.planePointCloudPtr);
  merge_grid.filter (mergeCloud);
  updatePlane.planePointCloudPtr->clear();
  *updatePlane.planePointCloudPtr = mergeCloud;

//  if(configPbMap.use_color)
//    updatePlane.calcMainColor();

  *discardPlane.polygonContourPtr += *updatePlane.planePointCloudPtr;
  updatePlane.calcConvexHull(discardPlane.polygonContourPtr);
  updatePlane.computeMassCenterAndArea();

  // Move the points to fulfill the plane equation
  updatePlane.forcePtsLayOnPlane();

  // Update area
  double area_recalc = updatePlane.planePointCloudPtr->size() * 0.0025;
  //mpPlaneInferInfo->isFullExtent(updatePlane, area_recalc);
  updatePlane.areaVoxels= updatePlane.planePointCloudPtr->size() * 0.0025;
}

void PbMapMaker::run()
{
   size_t numPrevKFs = 0;
   size_t minGrowPlanes = 5;
   while(!m_pbmaker_must_stop)  // Stop loop if PbMapMaker
   {
   if( numPrevKFs == frameQueue.size() )
     {
	   m_pbmaker_must_stop = true;
      //mrpt::system::sleep(10);
     }
     else
     {
    	cout<<"I enter the pbmaker run fun."<<endl;
    // Assign pointCloud of last KF to the global map
      //double time_start = pcl::getTime ();
      detectPlanesCloud(frameQueue.back().cloudPtr, frameQueue.back().pose,
                      configPbMap.dist_threshold, configPbMap.angle_threshold, configPbMap.minInliersRate);
      //double time_end = pcl::getTime ();
      //cout<<"Time consum is: "<<time_end-time_start<<endl;
       ++numPrevKFs;
     }
   }
  m_pbmaker_finished = true;
}

bool PbMapMaker::stop_pbMapMaker()
{
  m_pbmaker_must_stop = true;
  cout << "Waiting for PbMapMaker thread to die.." << endl;
  return true;
}

PbMapMaker::~PbMapMaker()
{

  stop_pbMapMaker();
  cout << " .. PbMapMaker has died." << endl;
}


