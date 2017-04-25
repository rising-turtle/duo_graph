#include "globaldef.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "octomap/Pointcloud.h"
#include "gicp/transform.h"
#include "ros/ros.h"
#include "timestamp.h"
// #include "FileReader.h"
#include <cmath>

/*
const double gl_gicp_epsilon = 1e-3;
const double gl_gicp_d_max_ = 5.0; // 10cm
const int gl_gicp_max_iterations = 50;
const int gl_gicp_min_point_cnt = 100;
const char* gl_pcd_file_dir = "/home/davidz/work/exprdata/pcds/conference_room";
*/

const char* gl_bag_color_name = "/camera/rgb/image_color";
const char* gl_bag_depth_name = "/camera/depth/image";
const char* gl_bag_camera_info = "/camera/depth/camera_info";
const char* gl_bag_tf_msg = "/tf";

static TTimeStamp gl_time_count_start;
static TTimeStamp gl_time_count_stop;

void StartTiming() // not thread-safe
{
    gl_time_count_start = getCurrentTime();
} 

double StopTiming()
{
    gl_time_count_stop = getCurrentTime();
    return timeDifference(gl_time_count_start, gl_time_count_stop);
}

void fromRot2RPY(double& roll, double& pitch, double& yaw, Eigen::Matrix3f& m_ROT )
{
	// Pitch is in the range [-pi/2, pi/2 ], so this calculation is enough:
	pitch =  atan2( (double)(- m_ROT(2,0)), (double)hypot( m_ROT(0,0),m_ROT(1,0) ) ); //asin( - m_ROT(2,0) );

	// Roll:
	if ( (fabs(m_ROT(2,1))+fabs(m_ROT(2,2)))<10*std::numeric_limits<double>::epsilon() )
	{
		//Gimbal lock between yaw and roll. This one is arbitrarily forced to be zero.
		//Check http://reference.mrpt.org/svn/classmrpt_1_1poses_1_1_c_pose3_d.html. If cos(pitch)==0, the homogeneous matrix is:
		//When sin(pitch)==1:
		//  /0  cysr-sycr cycr+sysr x\   /0  sin(r-y) cos(r-y)  x\.
		//  |0  sysr+cycr sycr-cysr y| = |0  cos(r-y) -sin(r-y) y|
		//  |-1     0         0     z|   |-1    0         0     z|
		//  \0      0         0     1/   \0     0         0     1/
		//
		//And when sin(pitch)=-1:
		//  /0 -cysr-sycr -cycr+sysr x\   /0 -sin(r+y) -cos(r+y) x\.
		//  |0 -sysr+cycr -sycr-cysr y| = |0 cos(r+y)  -sin(r+y) y|
		//  |1      0          0     z|   |1    0          0     z|
		//  \0      0          0     1/   \0    0          0     1/
		//
		//Both cases are in a "gimbal lock" status. This happens because pitch is vertical.

		roll = 0.0;
		if (pitch>0) yaw=atan2((double)m_ROT(1,2),(double)m_ROT(0,2));
		else yaw=atan2((double)(-m_ROT(1,2)),(double)(-m_ROT(0,2)));
	}
	else
	{
		roll = atan2( (double)(m_ROT(2,1)), (double)m_ROT(2,2) );
		// Yaw:
		yaw = atan2( (double)(m_ROT(1,0)), (double)(m_ROT(0,0)) );
	}
}




