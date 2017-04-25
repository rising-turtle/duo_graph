
#ifndef __PBMAPMAKER_H
#define __PBMAPMAKER_H

#include <EDPbmap.h>
#include <pcl/point_types.h>
#include <Plane.h>
#include <PbMap.h>
#include <pthread.h>
#define _VERBOSE 1

typedef pcl::PointXYZRGBA PointT;

  /*!frameRGBDandPose stores a dupla containing a pointCloud (built from a RGBD frame) and a pose.
   * \ingroup mrpt_pbmap_grp
   */
struct frameRGBDandPose
  {
    pcl::PointCloud<PointT>::Ptr cloudPtr;
    float pose[4][4];
  };
class PbMapMaker
{
  public:
  /*!PbMapMaker's constructor sets some threshold for plane segmentation and map growing from a configuration file (or default).
     This constructor also starts PbMapMaker's own thread.*/
    PbMapMaker(const std::string &config_file);

  /*!PbMapMaker's destructor is used to save some debugging info to file.*/
    ~PbMapMaker();

  /*!Get the PbMap.*/
    PbMap getPbMap(){return mPbMap;};

  /*!frameQueue is a vector containing the frameRGBDandPose (range image + pose) to be processed.*/
    std::vector<frameRGBDandPose> frameQueue;

  /*!observedPlanes is a list containing the current observed planes.*/
    std::set<unsigned> sQueueObservedPlanes;

    void run();


   private:

    /*!Find planar patches in the input organised point cloud "pointCloudPtr_arg", and update the PbMap with them (it update previous planes and
    initialize new ones when they are first observed), the input pose "poseInv" is used to place the current observations into a common frame of
    reference. Different thresholds are used to control the plane segmentation:
     - "distThreshold" defines the maximum distance of an inlier to the plane
     - "angleThreshold" defines the maximum angle between an inlier's normal and the plane's normal
     - "minInliersF" defines the minimum number of inliers as a fraction of the total number of points in the input cloud
     */
    void detectPlanesCloud( pcl::PointCloud<PointT>::Ptr &pointCloudPtr_arg, float inputpose[4][4], double distThreshold, double angleThreshold, double minInliersF);
    //void detectPlanesCloud( pcl::PointCloud<PointT>::Ptr &pointCloudPtr_arg, Eigen::Matrix4f &poseKF, double distThreshold, double angleThreshold, double minInliersF);

    /*!Returns true when the closest distance between the patches "plane1" and "plane2" is under distThreshold.*/
    bool arePlanesNearby(Plane &plane1, Plane &plane2, const float distThreshold);

    /*!Check for new graph connections of the input plane. These connections are stablished when the minimum distance between two patches is under
    the input threshold "proximity"*/
    void checkProximity(Plane &plane, float proximity);

    /*! Returns true if the two input planes represent the same physical surface for some given angle and distance thresholds.
     * If the planes are the same they are merged in this and the function returns true. Otherwise it returns false.*/
    bool areSamePlane(Plane &plane1, Plane &plane2, const float &cosAngleThreshold, const float &distThreshold, const float &proxThreshold);

    /*! Merge the two input patches into "updatePlane".
     *  Recalculate center, normal vector, area, inlier points (filtered), convex hull, etc.
     */
    void mergePlanes(Plane &updatePlane, Plane &discardPlane);

    /*!File containing some paramteres and heuristic thresholds"*/
    FILE *config_Param;



    protected:

    /*!The current PbMap.*/
    PbMap mPbMap;

    /*!List of planes observed in that last frame introduced.*/
    std::set<unsigned> observedPlanes;

    /*!This executes the PbMapMaker's thread*/
    //void run();

    /*!PbMapMaker's thread handle*/
    pthread_t pbmaker_hd;

    /*!Create the thread*/
    static void* ThreadPbMapMaker(void* lpParam);


    /*!PbMapMaker's exit thread*/
    bool stop_pbMapMaker();

    /*!PbMapMaker's stop controller*/
    bool	m_pbmaker_must_stop;

    /*!PbMapMaker's stop var*/
    bool	m_pbmaker_finished;
};

#endif
