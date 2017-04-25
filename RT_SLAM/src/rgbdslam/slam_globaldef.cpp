#include "slam_globaldef.h"

unsigned int gl_submap_size = 30; // 70
unsigned int gl_overlap_size = 5; // 10
unsigned int gl_start_dg_size = gl_submap_size - gl_overlap_size;
unsigned int gl_feature_observed_time = 1;

double gl_same_feature_dis = 0.2; 
const char* gl_submap_saved_path = "./submaps";
const char* gl_refined_submap_saved_path = "./refined";

float gl_pass_through_min_z = 0.5; // 0.5 m 0.5~6
float gl_pass_through_max_z = 6.; // 6 m 

float gl_octomap_voxel_size = 0.03; // voxel size 

unsigned int gl_min_inliers = 100; // 
