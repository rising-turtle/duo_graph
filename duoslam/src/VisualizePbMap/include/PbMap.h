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

#ifndef __PBMAP_H
#define __PBMAP_H

#include <EDPbmap.h>

#include <Plane.h>
#include <Miscellaneous.h>  // For typedef PointT;


	/** A class used to store a Plane-based Map (PbMap). A PbMap consists of a set of planar patches
	* described by geometric features (shape, relative position, etc.) and/or radiometric features
	* (dominant color). It is organized as an annotated, undirected graph, where nodes stand for planar
	* patches and edges connect neighbor planes when the distance between their closest points is under
	* a threshold. This graph structure permits to find efficiently the closest neighbors of a plane,
	* or to select groups of nearby planes representing part of the scene.
   *
   * \ingroup mrpt_pbmap_grp
   */
  class PbMap
  {

   public:
  /*!Constructor.*/
    PbMap();

    ~PbMap();

  /*!Vector to store the 3D-planes which are the basic characteristic of our map.*/
    std::vector<Plane> vPlanes;

  /*!Floor plane id*/
    int FloorPlane;

  /*!Registered point cloud from the RGB-D or Depth frames and visual odometry.*/
    pcl::PointCloud<PointT>::Ptr globalMapPtr;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr edgeCloudPtr;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr outEdgeCloudPtr;
    unsigned background, foreground, groundplane;

    /*!Merge two pbmaps*/
    void MergeWith(PbMap &pbm, Eigen::Matrix4f &T);

    /*! Print PbMap content to a text file*/
    void printPbMap(std::string txtFilePbm);

    /*! Load Pbmap txt into PbMap*/
    bool LoadPbMap(std::string txtFilePbm);
  };


#endif
