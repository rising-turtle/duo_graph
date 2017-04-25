#ifndef GLOBAL_DEF_H
#define GLOBAL_DEF_H
#include "signalslib.hpp"
#include <boost/shared_ptr.hpp>
#include <pcl/point_types.h>
#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include "gicp/transform.h"
#include "colortable.h"
#include "Eigen/Core"
#include <vector>
#include <string>
using namespace std;
// RAD2DEG
#define R2D(x) ((180.0*(x))/M_PI)
#define D2R(x) ((M_PI*(x))/180.0)

// SQUARE
#define SQ(x) ((x)*(x))

// gicp parameters
extern const double gl_gicp_epsilon;
extern const double gl_gicp_d_max_; // 10cm
extern const int gl_gicp_max_iterations;
extern const int gl_gicp_min_point_cnt;

// pcd file dir
extern const char* gl_pcd_file_dir;

// bag info 
extern const char* gl_bag_color_name;
extern const char* gl_bag_depth_name;
extern const char* gl_bag_camera_info;
extern const char* gl_bag_tf_msg;

extern void fromRot2RPY(double&, double&, double&, Eigen::Matrix3f&);

extern void StartTiming();
extern double StopTiming();

// extern void readPcdAndPose(const char*, std::vector<point_cloud_ptr>& , std::vector<octomap::pose6d>&, int num = -1, int step = 5);
#include "globaldef.hpp"

#endif
