#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include "ros/ros.h"
#include "NodeHolder.h"
#include "NodeWrapper.h"
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include "matching_result.h"
#include "RecePub.h"
#include "ReadFiles.h"
#include "GraphHolder.h"
#include "PbMap.h"
#include "PbMapVisualizer.h"

using namespace cv;
using namespace std;

void testVisPbmap();
void testGraphHolder();

/** @function main */
int main ( int argc, char** argv )
{
    ros::init(argc, argv, "pairwise");
    ros::NodeHandle nh;

    testVisPbmap();

    ros::spinOnce();
    usleep(100000);

    ROS_INFO("after runOffline!");
    while(ros::ok())
    {
        // ROS_INFO("Spin Once!");
        ros::spinOnce();
        usleep(100000);
    }
    return 0;
}

void testVisPbmap()
{
    string pcd_name("reconstructed_cloud.pcd");
    string pb_ini("Pbmap.txt");
    PbMapVisualizer mapViewer;
    pcl::PCDReader reader;
    reader.read(pcd_name, *mapViewer.pbmap.globalMapPtr);
    cout<<"pcd size: "<<mapViewer.pbmap.globalMapPtr->size()<<endl;
    if (!mapViewer.pbmap.LoadPbMap(pb_ini)) 
    {
        cout<<"failed to read pbMapFile: "<<pb_ini<<endl;
        return ;
    }
    mapViewer.Visualize();
}


void testGraphHolder()
{
    // CRecePub rece;
    string dir("/home/davidz/Data/rgbd_benchmark/rgbd_dataset_freiburg1_desk");
    // CReadFiles f(dir);
    // f.test();
    CGraphHolder g(dir);
    g.runOffline(10, -1, false);
    // g.runOfflineGT(10, 5, true);
}

/*
    void 
viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (1.0, 0.5, 1.0);
    pcl::PointXYZ o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere (o, 0.25, "sphere", 0);
    std::cout << "i only run once" << std::endl;
}
    camera_info_manager::CameraInfoManager info_mgr(nh, "rgb");
    sensor_msgs::CameraInfoPtr  info(new sensor_msgs::CameraInfo(info_mgr.getCameraInfo()));

    Mat img, depth; 
    img = imread("image_000420.png", -1);
    depth = imread("depth_000420.png", -1);
    
    Mat img2, depth2;
    img2 = imread("image_000455.png",-1);
    depth2 =imread("depth_000455.png", -1);
    unsigned int id = 0;
    std::string frame = "camera";
    ros::Time capture_time = ros::Time::now();

    std_msgs::Header header; 
    header.stamp = capture_time;
    header.seq = ++id;
    header.frame_id = frame;

    CNodeHolder node(img, depth, info, header);
    CNodeHolder pre_node(img2, depth2, info, header);
    pcl::io::savePCDFile("frame1.pcd", *(pre_node.getNode()->pc_col));
    pcl::io::savePCDFile("frame2.pcd", *(node.getNode()->pc_col));
    MatchingResult mr;
    node.matchNodePair(&pre_node, mr);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGB>);
    *cloud += *(pre_node.getNode()->pc_col);
    pcl::transformPointCloud(*(node.getNode()->pc_col), *tmp, mr.final_trafo);
    // tf::Transform tf = eigenTransf2TF(mr.final_trafo);
    // transformAndAppendPointCloud(*(node.getNode()->pc_col), *tmp, tf, 20); 
    *cloud += *tmp;
    pcl::io::savePCDFile("frame_align.pcd", *cloud);
    // pcl::io::loadPCDFile ("featurepcdtmp.pcd", *cloud);        

    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    // node.showPC(viewer);
    viewer.showCloud(cloud);
    // viewer.runOnVisualizationThreadOnce (viewerOneOff);
    
    while(!viewer.wasStopped())
    {
        // ROS_INFO("Spin Viewer");
    }
*/
 



/*
    /// Declare variables
    nfo_manager::CameraInfoManager info_mgr(nh, "rgb");
     42     sensor_msgs::CameraInfoPtr  info(new sensor_msgs::CameraInfo(info_mgr.getCameraInfo()));
     Maa src, dst;

    Mat kernel;
    Point anchor;
    double delta;
    int ddepth;
    int kernel_size;
    char* window_name = "filter2D Demo";

    int c;

    /// Load an image
    src = imread( argv[1] );

    if( !src.data )
    { return -1; }

    /// Create window
    namedWindow( window_name, CV_WINDOW_AUTOSIZE );

    /// Initialize arguments for the filter
    anchor = Point( -1, -1 );
    delta = 0;
    ddepth = -1;

    /// Loop - Will filter the image with different kernel sizes each 0.5 seconds
    int ind = 0;
    while( true )
    {
        c = waitKey(500);
        /// Press 'ESC' to exit the program
        if( (char)c == 27 )
        { break; }

        /// Update kernel size for a normalized box filter
        kernel_size = 3 + 2*( ind%5 );
        kernel = Mat::ones( kernel_size, kernel_size, CV_32F )/ (float)(kernel_size*kernel_size);

        /// Apply filter
        filter2D(src, dst, ddepth , kernel, anchor, delta, BORDER_DEFAULT );
        imshow( window_name, dst );
        ind++;
    }


*/
