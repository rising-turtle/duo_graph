/*
 * PbMapVisualizer.h
 *
 *  Created on: Nov 11, 2013
 *      Author: helen
 */

#ifndef PBMAPVISUALIZER_H_
#define PBMAPVISUALIZER_H_

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <EDPbmap.h>
#include <PbMap.h>

class PbMapVisualizer {
public:
	PbMapVisualizer();
	virtual ~PbMapVisualizer();


    pcl::visualization::CloudViewer cloudViewer;

    void Visualize();

    PbMap pbmap;

    void viz_cb (pcl::visualization::PCLVisualizer& viz);

 //   void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void);

};

#endif /* PBMAPVISUALIZER_H_ */

