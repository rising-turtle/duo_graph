#ifndef SLAM_GLOBALDEF_H
#define SLAM_GLOBALDEF_H

extern unsigned int gl_submap_size; // threshold 
extern unsigned int gl_overlap_size; // threshold 
extern unsigned int gl_start_dg_size;
extern unsigned int gl_feature_observed_time;

extern double gl_same_feature_dis; // threshold of distance for same features
extern const char* gl_submap_saved_path; // path to save the submaps
extern const char* gl_refined_submap_saved_path; // path to save the refined submaps

extern float gl_pass_through_min_z; // threshold for pass through filter
extern float gl_pass_through_max_z; 

extern float gl_octomap_voxel_size; // voxel size for octomap 

extern unsigned int gl_min_inliers; // the min number of the inliers between submapNode

#endif
