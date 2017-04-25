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
 *  Extract plane class from the pbmap by linyuan 2013-09-23
 */

#ifndef __EDPBMAP_PLANE_H
#define __EDPBMAP_PLANE_H

#include <EDPbmap.h>
#include <Miscellaneous.h>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>
#include <set>

	/** A class used to store a planar feature (Plane for short).
	 *  It is described with geometric features representing the shape and relative
	 *  location of the patch (area, normal vector, elongation, 3D-convex hull, etc.)
	 *  and radiometric features (the most representative color).
	 *
	 * \ingroup mrpt_pbmap_grp
	 */

static std::vector<size_t> DEFAULT_VECTOR;

class Plane 
{

 public:
 Plane() :
 elongation(1.0),
 bFullExtent(false),
 bFromStructure(false),
//      contourPtr(new pcl::PointCloud<pcl::PointXYZRGBA>),
 polygonContourPtr(new pcl::PointCloud<pcl::PointXYZRGBA>),
 planePointCloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA>)
 {
 }

/*!
* Force the plane inliers to lay on the plane
*/
 void forcePtsLayOnPlane();

/**!
* Calculate the plane's convex hull with the monotone chain algorithm.
*/
 void calcConvexHull(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &pointCloud, std::vector<size_t> &indices = DEFAULT_VECTOR );

/** \brief Compute the area of a 2D planar polygon patch - using a given normal
* \param polygonContourPtr the point cloud (planar)
* \param normal the plane normal
*/
 float compute2DPolygonalArea (/*pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &polygonContourPtr, Vector<3> &normal*/);

/** \brief Compute the patch's convex-hull area and mass center
*/
 void computeMassCenterAndArea();

/*!
* Calculate plane's elongation and principal direction
*/
 void calcElongationAndPpalDir();


/*!Returns true when the closest distance between the patches "this" and "plane" is under distThreshold.*/
bool isPlaneNearby(Plane &plane, const float distThreshold);

/*! Returns true if the two input planes represent the same physical surface for some given angle and distance thresholds.
* If the planes are the same they are merged in this and the function returns true. Otherwise it returns false.*/
bool isSamePlane(Plane &plane, const float &cosAngleThreshold, const float &distThreshold, const float &proxThreshold);

/*! Merge the two input patches into "updatePlane".
*  Recalculate center, normal vector, area, inlier points (filtered), convex hull, etc.
*/
 void mergePlane(Plane &plane);


/**!
*  Parameters to allow the plane-based representation of the map by a graph
*/
 unsigned id;
 unsigned numObservations;
 unsigned semanticGroup;
 std::set<unsigned> nearbyPlanes;
 std::map<unsigned,unsigned> neighborPlanes;
 std::string label;

/**!
*  Geometric description
*/
 Eigen::Vector3f v3center;
 Eigen::Vector3f v3normal;
 Eigen::Vector3f v3PpalDir;
 float elongation; // This is the reatio between the lengths of the plane in the two principal directions
 float areaVoxels;
 float areaHull;
 bool bFullExtent;
 bool bFromStructure;
 unsigned nFramesAreaIsStable;

/**!
*  Radiometric description
*/
 Eigen::Vector3f v3colorNrgb;
 Eigen::Vector3f v3colorNrgbDev;

/**!
*  Convex Hull
*/
//    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr contourPtr;
 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr polygonContourPtr;
 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr outerPolygonPtr; // This is going to be deprecated
 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planePointCloudPtr; // This is going to be deprecated

/*!
* Calculate plane's main color using "MeanShift" method
*/
 void calcMainColor();


 private:
/*!
* Calculate plane's main color in C1C2C3 representation
*/
 void getPlaneNrgb();

 std::vector<float> r;
 std::vector<float> g;
 std::vector<float> b;

};

#endif
