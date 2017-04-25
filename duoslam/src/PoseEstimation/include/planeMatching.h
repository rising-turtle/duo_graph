/*
 * planeMatching.h
 *
 *  Created on: Nov 5, 2013
 *      Author: helen
 */

#ifndef PLANEMATCHING_H_
#define PLANEMATCHING_H_

#include <EDPbmap.h>
#include <Plane.h>
#include <PbMap.h>

 class PlaneMatching
  {
 	 public:

	    /*!Constructor */
	 	 PlaneMatching(PbMap &PBM_source, PbMap &PBM_target);

		/*!List of pairs of matched planes from the PbMaps PBMSource with those from PBMTarget*/
		 std::map<unsigned, unsigned> matched_planes;


 	 private:
	    /*!One of the subgraphs matched by SubgraphMatcher.*/
	    PbMap &PBMSource;

	    /*!The other subgraph matched by SubgraphMatcher.*/
	    PbMap &PBMTarget;


	    void findMaxIndex(int* vote, int num, int* max_index, int* max_num, int& max_size);

	    void findMatchedPlanes(std::vector<Plane>& planes_src, std::vector<Plane>& planes_target, std::map<unsigned, unsigned> &match_res);

  };


#endif /* PLANEMATCHING_H_ */
