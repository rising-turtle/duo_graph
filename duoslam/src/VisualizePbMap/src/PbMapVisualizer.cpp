/*
 * PbMapVisualizer.cpp
 *
 *  Created on: Nov 11, 2013
 *      Author: helen
 */

#include "PbMapVisualizer.h"
#include "Plane.h"
using namespace std;

// Color = (red[i], grn[i], blu[i])
// The color order is: red, green, blue, yellow, pink, turquoise, orange, purple, dark green, beige
unsigned char red [10] = {255,   0,   0, 255, 255,   0, 255, 204,   0, 255};
unsigned char grn [10] = {  0, 255,   0, 255,   0, 255, 160,  51, 128, 222};
unsigned char blu [10] = {  0,   0, 255,   0, 255, 255, 0  , 204,   0, 173};

double ared [10] = {1.0,   0,   0, 1.0, 1.0,   0, 1.0, 0.8,   0, 1.0};
double agrn [10] = {  0, 1.0,   0, 1.0,   0, 1.0, 0.6, 0.2, 0.5, 0.9};
double ablu [10] = {  0,   0, 1.0,   0, 1.0, 1.0,   0, 0.8,   0, 0.7};

bool graphRepresentation;
bool showPointCloud;
bool showPbMap;


PbMapVisualizer::PbMapVisualizer():
    	    cloudViewer("PbMap"){
	// TODO Auto-generated constructor stub
    graphRepresentation = false;
    showPointCloud = true;
    showPbMap = true;

}

PbMapVisualizer::~PbMapVisualizer() {
	// TODO Auto-generated destructor stub
}

void PbMapVisualizer::viz_cb (pcl::visualization::PCLVisualizer& viz)
{
  if (pbmap.globalMapPtr->empty())
  {
    return;
  }

  // Render the data
  {
    viz.removeAllShapes();
    viz.removeAllPointClouds();

    char name[1024];

    if(graphRepresentation)
    {
//      cout << "show graphRepresentation\n";
      for(size_t i=0; i<pbmap.vPlanes.size(); i++)
      {
        pcl::PointXYZ center(2*pbmap.vPlanes[i].v3center[0], 2*pbmap.vPlanes[i].v3center[1], 2*pbmap.vPlanes[i].v3center[2]);
        double radius = 0.1 * sqrt(pbmap.vPlanes[i].areaVoxels);
//        cout << "radius " << radius << endl;
        sprintf (name, "sphere%u", static_cast<unsigned>(i));
        viz.addSphere (center, radius, ared[i%10], agrn[i%10], ablu[i%10], name);

        if( !pbmap.vPlanes[i].label.empty() )
            viz.addText3D (pbmap.vPlanes[i].label, center, 0.1, ared[i%10], agrn[i%10], ablu[i%10], pbmap.vPlanes[i].label);
        else
        {
          sprintf (name, "P%u", static_cast<unsigned>(i));
          viz.addText3D (name, center, 0.1, ared[i%10], agrn[i%10], ablu[i%10], name);
        }

        // Draw edges
//        for(set<unsigned>::iterator it = pbmap.vPlanes[i].nearbyPlanes.begin(); it != pbmap.vPlanes[i].nearbyPlanes.end(); it++)
//        {
//          if(*it > pbmap.vPlanes[i].id)
//            break;
//
//          sprintf (name, "commonObs%u_%u", static_cast<unsigned>(i), static_cast<unsigned>(*it));
//          pcl::PointXYZ center_it(2*pbmap.vPlanes[*it].v3center[0], 2*pbmap.vPlanes[*it].v3center[1], 2*pbmap.vPlanes[*it].v3center[2]);
//          viz.addLine (center, center_it, ared[i%10], agrn[i%10], ablu[i%10], name);
//        }
        for(map<unsigned,unsigned>::iterator it = pbmap.vPlanes[i].neighborPlanes.begin(); it != pbmap.vPlanes[i].neighborPlanes.end(); it++)
        {
          if(it->first > pbmap.vPlanes[i].id)
            break;

          sprintf (name, "commonObs%u_%u", static_cast<unsigned>(i), static_cast<unsigned>(it->first));
          pcl::PointXYZ center_it(2*pbmap.vPlanes[it->first].v3center[0], 2*pbmap.vPlanes[it->first].v3center[1], 2*pbmap.vPlanes[it->first].v3center[2]);
          viz.addLine (center, center_it, ared[i%10], agrn[i%10], ablu[i%10], name);

          sprintf (name, "edge%u_%u", static_cast<unsigned>(i), static_cast<unsigned>(it->first));
          char commonObs[8];
          sprintf (commonObs, "%u", it->second);
          pcl::PointXYZ half_edge( (center_it.x+center.x)/2, (center_it.y+center.y)/2, (center_it.z+center.z)/2 );
          viz.addText3D (commonObs, half_edge, 0.05, 1.0, 1.0, 1.0, name);
        }
      }
    }
    else
    { // Regular representation
      if (!viz.updatePointCloud (pbmap.globalMapPtr, "cloud"))
        viz.addPointCloud (pbmap.globalMapPtr, "cloud");

      sprintf (name, "PointCloud size %u", static_cast<unsigned>( pbmap.globalMapPtr->size() ) );
      viz.addText(name, 10, 20);

      for(size_t i=0; i<pbmap.vPlanes.size(); i++)
      {
        Plane &plane_i = pbmap.vPlanes[i];
        sprintf (name, "normal_%u", static_cast<unsigned>(i));
        //       name[0] = *(mrpt::format("normal_%u", static_cast<unsigned>(i)).c_str());
        pcl::PointXYZ pt1, pt2; // Begin and end points of normal's arrow for visualization
        pt1 = pcl::PointXYZ(plane_i.v3center[0], plane_i.v3center[1], plane_i.v3center[2]);
        pt2 = pcl::PointXYZ(plane_i.v3center[0] + (0.5f * plane_i.v3normal[0]),
                            plane_i.v3center[1] + (0.5f * plane_i.v3normal[1]),
                            plane_i.v3center[2] + (0.5f * plane_i.v3normal[2]));
        viz.addArrow (pt2, pt1, ared[i%10], agrn[i%10], ablu[i%10], false, name);

        if( !plane_i.label.empty() )
          viz.addText3D (plane_i.label, pt2, 0.1, ared[i%10], agrn[i%10], ablu[i%10], plane_i.label);
        else
        {
          sprintf (name, "n%u", static_cast<unsigned>(i));
          viz.addText3D (name, pt2, 0.1, ared[i%10], agrn[i%10], ablu[i%10], name);
        }

        sprintf (name, "approx_plane_%02d", int (i));
        viz.addPolygon<PointT> (plane_i.polygonContourPtr, 0.5 * red[i%10], 0.5 * grn[i%10], 0.5 * blu[i%10], name);
      }
    }
  }
}


void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
  if ( (event.getKeySym () == "r" || event.getKeySym () == "R") && event.keyDown ())
  {
    cout << "r was pressed => change between regular/graph representation" << endl;
    graphRepresentation = !graphRepresentation;
  }
  else if ( (event.getKeySym () == "p" || event.getKeySym () == "P") && event.keyDown ())
    showPointCloud = !showPointCloud;
  else if ( (event.getKeySym () == "l" || event.getKeySym () == "L") && event.keyDown ())
    showPbMap = !showPbMap;
}

void PbMapVisualizer::Visualize()
{
  cloudViewer.runOnVisualizationThread (boost::bind(&PbMapVisualizer::viz_cb, this, _1), "viz_cb");
  cloudViewer.registerKeyboardCallback (keyboardEventOccurred);

  while (!cloudViewer.wasStopped() )
    //mrpt::system::sleep(10);
	  usleep(10000);
}


