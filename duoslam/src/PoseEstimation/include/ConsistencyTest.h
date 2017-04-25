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

#ifndef __CONSISTENCYTEST_H
#define __CONSISTENCYTEST_H

#include <EDPbmap.h>
#include <PbMap.h>

#define _VERBOSE 1


  /*! This class computes the rigid transformation between two sets of matched planes,
   *  and provides a measure of their rigid adjustment
   *
   * \ingroup mrpt_pbmap_grp
   */
  class ConsistencyTest
  {
   public:

    /*!Constructor */
    ConsistencyTest(PbMap &PBM_source, PbMap &PBM_target);

  //  /**! Get diamond of points around the center. This is used to calculate the adjustment error with a model plane */
  //  void calcDiamondPlane(Plane& plane);

    /*!Calculate the alignment error between two sets of matched planes.
      The input rigid transformation "se3rigidTransfInv" is used to project the centroids of one set of planes into their
      matched planes and returns the sum of cuadratic distances */
    double calcAlignmentError( std::map<unsigned, unsigned> &matched_planes, Eigen::Matrix4f &rigidTransf );

    /*!Return an initial guess for the rigid transformation which aligns two matched places.
    The translation is calculated from the planes centroids and the rotation from the alignment of the plane's normals.*/
    Eigen::Matrix4f initPose( std::map<unsigned, unsigned> &matched_planes);

    /*!Return an initial guess for the rigid transformation which aligns two matched places.
    The translation is calculated from the planes centroids and the rotation from the alignment of the plane's normals.*/
    Eigen::Matrix4f initPoseLY( std::map<unsigned, unsigned> &matched_planes);

    /*!Return an initial guess for the rigid transformation which aligns two matched places.
    The translation is calculated from the planes centroids and the rotation from the alignment of the plane's normals.
    A planar movement is assumed (wheeled robot)*/
    Eigen::Matrix4f initPose2D( std::map<unsigned, unsigned> &matched_planes);

    /*!Return the estimated rigid transformation which aligns two matched subgraphs (i.e. neighborhoods of planes).
    This function iteratively minimizes the alignment error of the matched planes wrt the rigid transformation.*/
    Eigen::Matrix4f getRTwithModel( std::map<unsigned, unsigned> &matched_planes );

    /*!Return the estimated rigid transformation which aligns two matched subgraphs (i.e. neighborhoods of planes).
    This function iteratively minimizes the alignment error of the matched planes wrt the rigid transformation.*/
    Eigen::Matrix4f getRTwithModelLY( std::map<unsigned, unsigned> &matched_planes );


   private:

    /*!One of the subgraphs matched by SubgraphMatcher.*/
    PbMap &PBMSource;

    /*!The other subgraph matched by SubgraphMatcher.*/
    PbMap &PBMTarget;

    /*!List of pairs of matched planes from the PbMaps PBMSource with those from PBMTarget*/
    std::map<unsigned, unsigned> matched_planes;


    void updateposeFUN(Eigen::Matrix<float,6,1> updatedSE3, Eigen::Matrix4f &updatePose);
  };

#endif

