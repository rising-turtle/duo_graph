#include "GraphHolder.h"
#include "NodeWrapper.h"
#include "ReadFiles.h"
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include "pcl/filters/voxel_grid.h"
#include "matching_result.h"
#include "pcl_ros/transforms.h"
// #include "opencv/cv.h"

namespace{
    void transformAndAppendPointCloud (const pointcloud_type &cloud_in, 
            pointcloud_type &cloud_to_append_to,
            const tf::Transform transformation, float max_Depth)
    {
        bool compact = true; // !ParameterServer::instance()->get<bool>("preserve_raster_on_save");
        Eigen::Matrix4f eigen_transform;
        pcl_ros::transformAsMatrix(transformation, eigen_transform);
        unsigned int cloud_to_append_to_original_size = cloud_to_append_to.size();
        if(cloud_to_append_to.points.size() ==0)
        {
            cloud_to_append_to.header   = cloud_in.header;
            cloud_to_append_to.width    = 0;
            cloud_to_append_to.height   = 0;
            cloud_to_append_to.is_dense = false;
        }

        //Append all points untransformed
        cloud_to_append_to += cloud_in;

        Eigen::Matrix3f rot   = eigen_transform.block<3, 3> (0, 0);
        Eigen::Vector3f trans = eigen_transform.block<3, 1> (0, 3);
        point_type origin = point_type();
        origin.x = 0;
        origin.y = 0;
        origin.z = 0;
        int j = 0;
        for (size_t i = 0; i < cloud_in.points.size (); ++i)
        { 
            Eigen::Map<Eigen::Vector3f> p_in (const_cast<float*>(&cloud_in.points[i].x), 3, 1);
            Eigen::Map<Eigen::Vector3f> p_out (&cloud_to_append_to.points[j+cloud_to_append_to_original_size].x, 3, 1);
            if(compact){ cloud_to_append_to.points[j+cloud_to_append_to_original_size] = cloud_in.points[i]; }
            //filter out points with a range greater than the given Parameter or do nothing if negativ
            if(max_Depth >= 0){
                if(pcl::squaredEuclideanDistance(cloud_in.points[i], origin) > max_Depth*max_Depth){
                    p_out[0]= std::numeric_limits<float>::quiet_NaN();
                    p_out[1]= std::numeric_limits<float>::quiet_NaN();
                    p_out[2]= std::numeric_limits<float>::quiet_NaN();
                    if(!compact) j++; 
                    continue;
                }
            }
            if (pcl_isnan (cloud_in.points[i].x) || pcl_isnan (cloud_in.points[i].y) || pcl_isnan (cloud_in.points[i].z)){
                if(!compact) j++;
                continue;
            }
            p_out = rot * p_in + trans;
            j++;
        }
        if(compact){
            cloud_to_append_to.points.resize(j+cloud_to_append_to_original_size);
            cloud_to_append_to.width    = 1;
            cloud_to_append_to.height   = j+cloud_to_append_to_original_size;
        }
    }
    void VoxelFilter(pointcloud_ptr input_pc, double leaf_size)
    {
        //filterqPrintable()
        pcl::VoxelGrid<point_type> voxel_grid;
        voxel_grid.setInputCloud (input_pc->makeShared());
        voxel_grid.setLeafSize (leaf_size, leaf_size, leaf_size);
        pcl::PointCloud<point_type>::Ptr downsampled (new pcl::PointCloud<point_type>);
        voxel_grid.filter (*downsampled);
        input_pc.swap(downsampled);
    }
    void VoxelFilter(pointcloud_ptr input_pc, pointcloud_ptr output_pc, double leaf_size)
    {
        pcl::VoxelGrid<point_type> voxel_grid;
        voxel_grid.setInputCloud(input_pc->makeShared());
        voxel_grid.setLeafSize (leaf_size, leaf_size, leaf_size);
        voxel_grid.filter(*output_pc);
    }
}

CGraphHolder::CGraphHolder(string dir):
m_pRecords(new CReadFiles(dir))
{
    ros::NodeHandle nh;
    camera_info_manager::CameraInfoManager info_mgr(nh, "rgb");     
    sensor_msgs::CameraInfoPtr info(new sensor_msgs::CameraInfo(info_mgr.getCameraInfo()));
    m_camInfo = info;
}

CGraphHolder::~CGraphHolder()
{
    if(m_pRecords)
    {
        delete m_pRecords;
        m_pRecords = 0;
    }
}

void CGraphHolder::parseGT(tf::Transform& gt, string gt_v)
{
    tf::Vector3 t;
    tf::Quaternion r;
    double x,y,z,w; 
    cerr<<"GraphHolder.cpp : "<<gt_v<<endl;
    sscanf(gt_v.c_str(), "%lf %lf %lf %lf %lf %lf %lf", &t.m_floats[0], &t.m_floats[1], &t.m_floats[2], &x, &y, &z, &w);
   //                             &(r.x()), &(r.y()), &(r.z()), &(r.w()));
    r.setValue(x,y,z,w);
    t.m_floats[3] = 0;
    gt = tf::Transform(r,t);
}

void CGraphHolder::dumpTrans2File(ostream& out, tf::Transform& trans, double timestamp)
{
    tf::Vector3 t = trans.getOrigin(); 
    tf::Quaternion r = trans.getRotation();
    out<<fixed<<timestamp<<" "<<t.getX()<<" "<<t.getY()<<" "<<t.getZ()<<" ";
    out<<r.getX()<<" "<<r.getY()<<" "<<r.getZ()<<" "<<r.getW()<<endl;
}

tf::Transform CGraphHolder::eigen2TF(Eigen::Matrix4f& tf)
{
    // tf::Transform ret_tf; 
    tf::Matrix3x3 R; 
    R.setValue(tf(0,0), tf(0,1), tf(0,2), \
               tf(1,0), tf(1,1), tf(1,2), \
               tf(2,0), tf(2,1), tf(2,2));
    tf::Vector3 T; 
    T.setValue(tf(0,3), tf(1,3), tf(2,3));
    return tf::Transform(R, T);
}

void CGraphHolder::fusePC(pointcloud_ptr pc_all, pointcloud_ptr pc_append, tf::Transform tf)
{
     pointcloud_ptr tmp(new pointcloud_type);
     transformAndAppendPointCloud(*pc_append, *tmp, tf, -1); 
     *pc_all += *tmp;
}

void CGraphHolder::setPCDPose(pointcloud_ptr pc, tf::Transform tf)
{
    tf::Vector3 t = tf.getOrigin();
    tf::Quaternion q = tf.getRotation();
    Eigen::Vector4f et;
    Eigen::Quaternionf eq(q.w(), q.x(), q.y(), q.z());
    et(0) = t.x(); et(1) = t.y(); et(2) = t.z(); et(3) = 0.;
    pc->sensor_origin_ = et;
    pc->sensor_orientation_ = eq;
}

void CGraphHolder::runOfflineGT2(int step, int num, bool use_pc)
{
    CNodeHolder* pN = 0; 
    bool bCounting = num > 0; 
    int n_cout = 0;

    string rgbf, dptf, gtf; 
    ofstream gt_traj("gt_trajetory.log");
    pointcloud_ptr gt_pc(new pointcloud_type);

    tf::Transform gt_tf; 
    double timestamp; 
    string gt_v;
    
    int i =0;
    string pcd_name;

    while(m_pRecords->readRecord(rgbf, dptf, gtf))
    {
        m_pRecords->getCurGtStr(timestamp, gt_v);
        parseGT(gt_tf, gt_v); 
        dumpTrans2File(gt_traj, gt_tf, timestamp);            

        if( i<step && i!=0) { ++i; continue;}
        else i=1;

        if(use_pc)
        {
            if(pN != 0) delete pN;
            pN = createNode(rgbf, dptf);
            
            // tf::Transform t = ini_tf.inverse() * gt_tf;
            // tf::Transform t = gt_tf * ini_tf.inverse();
            // fusePC(gt_pc, pN->getNode()->pc_col, t);
            gt_pc = pN->getNode()->pc_col->makeShared();
            VoxelFilter(gt_pc, 0.05);
            setPCDPose(gt_pc, gt_tf);
            stringstream ss_pcd;
            ss_pcd<<"gt_pcd"<<n_cout+1<<".pcd";
            pcl::io::savePCDFile(ss_pcd.str().c_str(), *gt_pc);
        }
        if(bCounting && n_cout >= num)
        {
            cout<<"GraphHolder.cpp finish !"<<endl;
            break;
        }
        n_cout++;
    }
    if(use_pc)
    {
        VoxelFilter(gt_pc, 0.1); 
        pcl::io::savePCDFile("all_gt2.pcd", *gt_pc);
    }

}

void CGraphHolder::runOfflineGT(int step, int num, bool use_pc)
{
    CNodeHolder* pN = 0; 
    bool bCounting = num > 0; 
    int n_cout = 0;

    string rgbf, dptf, gtf; 
    ofstream gt_traj("gt_trajetory.log");
    pointcloud_ptr gt_pc(new pointcloud_type);

    tf::Transform gt_tf; 
    tf::Transform ini_tf; 
    bool bfirst = true;
    double timestamp; 
    string gt_v;
    
    int i =0;

    while(m_pRecords->readRecord(rgbf, dptf, gtf))
    {
        m_pRecords->getCurGtStr(timestamp, gt_v);
        parseGT(gt_tf, gt_v); 
        dumpTrans2File(gt_traj, gt_tf, timestamp);            

        if( i<step && i!=0) { ++i; continue;}
        else i=1;

        if(bfirst)
        {
            ini_tf = gt_tf;
            bfirst = false;
        }
        if(use_pc)
        {
            if(pN != 0) delete pN;
            pN = createNode(rgbf, dptf);
            tf::Transform t = ini_tf.inverse() * gt_tf;
            // tf::Transform t = gt_tf * ini_tf.inverse();
            fusePC(gt_pc, pN->getNode()->pc_col, t);
            VoxelFilter(gt_pc, 0.05);
        }
        if(bCounting && n_cout >= num)
        {
            cout<<"GraphHolder.cpp finish !"<<endl;
            break;
        }
        n_cout++;
    }
    if(use_pc)
    {
        VoxelFilter(gt_pc, 0.1); 
        pcl::io::savePCDFile("all_gt2.pcd", *gt_pc);
    }
}

void CGraphHolder::runOffline(int step, int num, bool use_pc)
{
    int i = 0; 
    bool bfirst = true;
    CNodeHolder* pN1 = 0;
    CNodeHolder* pN2 = 0;
    ofstream estimate_traj("/home/davidz/Downloads/rgbd_tools/est_trajectory.txt");
    ofstream gt_traj("/home/davidz/Downloads/rgbd_tools/gt_trajetory.txt");
    tf::Transform est_tf;
    tf::Transform gt_tf;
    string gt_v; 
    double timestamp;
    string rgbf, dptf, gtf;
    
    pointcloud_ptr gt_pc(new pointcloud_type);
    pointcloud_ptr es_pc(new pointcloud_type);

    bool bCounting = (num > 0);
    int n_cout = 0;

    char input_c;
    bool step_control = true;
    // test phase
    while(m_pRecords->readRecord(rgbf, dptf, gtf))
    {
        if(bfirst)
        {
            pN2 = createNode(rgbf, dptf); 
            if(!m_pRecords->getCurGtStr(timestamp, gt_v))
            {
                cerr<<"GraphHolder.cpp: getCurGtStr err!"<<endl;
                break;
            }
            parseGT(gt_tf, gt_v);
            est_tf = gt_tf;
            dumpTrans2File(estimate_traj, est_tf, m_pRecords->getCurQueryTime());
            dumpTrans2File(gt_traj, gt_tf, timestamp);            
            bfirst = false;
            if(use_pc)
            {
                fusePC(es_pc, pN2->getNode()->pc_col, est_tf);
                fusePC(gt_pc, pN2->getNode()->pc_col, gt_tf);
                VoxelFilter(es_pc, 0.05); 
                VoxelFilter(gt_pc, 0.05);
            } 
            continue;
        }
        if(i++ < step) continue; 
        else i = 0;
        
        if(step_control)
        {
            cout<<"GraphHolder.cpp: next command: "<<endl;
            cout<<"S: stop; C: without querying, else: next."<<endl;
            cin>>input_c;
            if(input_c == 'S' or input_c == 's')
            {
                cout<<"Stop !"<<endl;
                break;
            }else if(input_c == 'C' or input_c == 'c')
            {
                cout<<"Without stop!"<<endl;
                step_control = false;
            }
        }
        if(pN1 !=0 ) delete pN1; 
        pN1 = pN2;
        pN2 = createNode(rgbf, dptf);
        
        CNodeHolder* pN2 = createNode(rgbf, dptf);
        MatchingResult mr;
        pN2->matchNodePair(pN1, mr);

        if(mr.inlier_matches.size() <= 0)
        {
            cerr<<"failed to align frame: "<<rgbf<<endl;
            i = step;
            continue;
        }
        
        // tf::Transform cur_trans = eigenTransf2TF(mr.final_trafo);
        tf::Transform cur_trans = eigen2TF(mr.final_trafo);
        est_tf = est_tf * cur_trans;
        if(!m_pRecords->getCurGtStr(timestamp, gt_v))
        {
            cerr<<"GraphHolder.cpp: getCurGtStr err!"<<endl;
            break;
        }
        parseGT(gt_tf, gt_v);

        dumpTrans2File(estimate_traj, est_tf, m_pRecords->getCurQueryTime());
        dumpTrans2File(gt_traj, gt_tf, timestamp);

        if(bCounting && n_cout >= num)
        {
            cerr<<"GraphHolder.cpp: has already finished all frames!"<<endl;
            break;
        }
        ++n_cout;
        
        if(use_pc)
        {
            fusePC(es_pc, pN2->getNode()->pc_col, est_tf);
            fusePC(gt_pc, pN2->getNode()->pc_col, gt_tf);
            VoxelFilter(es_pc, 0.05); 
            VoxelFilter(gt_pc, 0.05);
        }

        // pcl::io::savePCDFile("test1.pcd", *(pN1->getNode()->pc_col));
        // pcl::io::savePCDFile("test2.pcd", *(pN2->getNode()->pc_col));
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGB>);
        // *cloud += *(pN1->getNode()->pc_col);
        // pcl::transformPointCloud(*(pN2->getNode()->pc_col), *tmp, mr.final_trafo);
        // tf::Transform tf = eigenTransf2TF(mr.final_trafo);
        // transformAndAppendPointCloud(*(node.getNode()->pc_col), *tmp, tf, 20); 
        // *cloud += *tmp;
        // pcl::io::savePCDFile("frame_align.pcd", *cloud);
    }
    if(use_pc)
    {
        // VoxelFilter(es_pc, 0.05); 
        // VoxelFilter(gt_pc, 0.05);
        pcl::io::savePCDFile("all_estimate.pcd", *es_pc);
        pcl::io::savePCDFile("all_groundtruth.pcd", *gt_pc);
    }
}

CNodeHolder* CGraphHolder::createNode(string rgbf, string dptf)
{
    static unsigned int id = 0;
    static string frame = "/camera";
    ros::Time timestamp = ros::Time::now();
    std_msgs::Header h ;
    h.stamp = timestamp;
    h.seq = ++id;
    h.frame_id = frame;
    cv::Mat img, dpt;
    // cout<<"rgb path: "<<rgbf<<endl;
    // cout<<"dpt path: "<<dptf<<endl;
    img = cv::imread(rgbf.c_str(), -1);
    dpt = cv::imread(dptf.c_str(), -1);
    
    CNodeHolder* pN = new CNodeHolder(img, dpt, m_camInfo, h);
    return pN;
}


