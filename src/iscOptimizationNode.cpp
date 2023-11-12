// Author of INTENSITY-SLAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include <iostream>
#include <string>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <fstream>
//ros
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>

//PCL
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//opencv

//my library
#include "iscOptimizationClass.h"
#include <intensity_slam/LoopInfo.h>

ros::Publisher odom_pub;
ros::Publisher path_pub;
ros::Publisher loop_map_pub;
ros::Publisher loop_candidate_pub;
std::mutex mutex_lock;

//queue for loop closure detection
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudSurfBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudEdgeBuf;
std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf;
std::queue<intensity_slam::LoopInfo::ConstPtr> loopInfoBuf;

ISCOptimizationClass iscOptimization;

double scan_period= 0.1;


Eigen::Isometry3d w_odom_curr= Eigen::Isometry3d::Identity();
nav_msgs::Path path_optimized;
int current_frame_id;
std::vector<int> matched_frame_id;
//receive odomtry
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    mutex_lock.lock();
    odometryBuf.push(msg);
    mutex_lock.unlock();

    // high frequence publish
    Eigen::Isometry3d odom_to_base= Eigen::Isometry3d::Identity();
    odom_to_base.rotate(Eigen::Quaterniond(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z));  
    odom_to_base.pretranslate(Eigen::Vector3d(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z));
    Eigen::Isometry3d w_curr = w_odom_curr*odom_to_base;
    Eigen::Quaterniond q_temp(w_curr.rotation());

    static tf::TransformBroadcaster br_realtime;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(w_curr.translation().x(), w_curr.translation().y(), w_curr.translation().z()) );
    tf::Quaternion q(q_temp.x(),q_temp.y(),q_temp.z(),q_temp.w());
    transform.setRotation(q);
    br_realtime.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/base_link"));

    // publish odometry
    nav_msgs::Odometry laserOdometry;
    laserOdometry.header.frame_id = "/map"; //world
    laserOdometry.child_frame_id = "/base_link"; //odom
    laserOdometry.header.stamp = msg->header.stamp;
    laserOdometry.pose.pose.orientation.x = q_temp.x();
    laserOdometry.pose.pose.orientation.y = q_temp.y();
    laserOdometry.pose.pose.orientation.z = q_temp.z();
    laserOdometry.pose.pose.orientation.w = q_temp.w();
    laserOdometry.pose.pose.position.x = w_curr.translation().x();
    laserOdometry.pose.pose.position.y = w_curr.translation().y();
    laserOdometry.pose.pose.position.z = w_curr.translation().z();
    odom_pub.publish(laserOdometry);
}

//receive all point cloud
void pointCloudSurfCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    mutex_lock.lock();
    pointCloudSurfBuf.push(msg);
    mutex_lock.unlock();
}
//receive all point cloud
void pointCloudEdgeCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    mutex_lock.lock();
    pointCloudEdgeBuf.push(msg);
    mutex_lock.unlock();
}

//receive all point cloud
void loopClosureCallback(const intensity_slam::LoopInfoConstPtr &msg)
{
    mutex_lock.lock();
    loopInfoBuf.push(msg);
    mutex_lock.unlock();
}

//TODO_czy
bool CreateFile(std::ofstream& ofs, std::string file_path) {
    ofs.open(file_path.c_str(), std::ios::app);
    if (!ofs) {
       std::cout << "无法生成文件: " << file_path << std::endl;
        return false;
    }

    return true;
}
//TODO_czy
bool CreateDirectory(std::string directory_path) {
    if (!boost::filesystem::is_directory(directory_path)) {
        boost::filesystem::create_directory(directory_path);
    }
    if (!boost::filesystem::is_directory(directory_path)) {
        std::cout << "无法建立文件夹: " << directory_path << std::endl;
        return false;
    }
    return true;
}
//TODO_czy
std::string WORK_SPACE_PATH = "/home/bupo/my_study/ig_lio_git/intensity_slam";

int frmae_count =0;
Eigen::Isometry3d transform_to_base;
std::string file_name;
void global_optimization(){
    while(1){
        if(!loopInfoBuf.empty()&& !pointCloudSurfBuf.empty() && !pointCloudEdgeBuf.empty() && !odometryBuf.empty()){

            mutex_lock.lock();

            double time1 =  loopInfoBuf.front()->header.stamp.toSec();
            double time2 =  odometryBuf.front()->header.stamp.toSec();
            double time3 =  pointCloudEdgeBuf.front()->header.stamp.toSec();
            double time4 =  pointCloudSurfBuf.front()->header.stamp.toSec();
            if(!loopInfoBuf.empty() && (time1<time2-0.5*scan_period || time1<time3-0.5*scan_period || time1<time4-0.5*scan_period)){
                ROS_WARN("time stamp unaligned error and loopInfoBuf discarded, pls check your data --> isc optimization"); 
                loopInfoBuf.pop();
                mutex_lock.unlock();
                continue;              
            }
            if(!odometryBuf.empty() && (time2<time1-0.5*scan_period || time2<time3-0.5*scan_period || time2<time4-0.5*scan_period)){
                ROS_WARN("time stamp unaligned error and odometryBuf discarded, pls check your data --> isc optimization"); 
                odometryBuf.pop();
                mutex_lock.unlock();
                continue;              
            }
            if(!pointCloudEdgeBuf.empty() && (time3<time1-0.5*scan_period || time3<time2-0.5*scan_period || time3<time4-0.5*scan_period)){
                ROS_WARN("time stamp unaligned error and pointCloudEdgeBuf discarded, pls check your data --> isc optimization"); 
                pointCloudEdgeBuf.pop();
                mutex_lock.unlock();
                continue;              
            }
            if(!pointCloudSurfBuf.empty() && (time4<time1-0.5*scan_period || time4<time2-0.5*scan_period || time4<time3-0.5*scan_period)){
                ROS_WARN("time stamp unaligned error and pointCloudSurfBuf discarded, pls check your data --> isc optimization"); 
                pointCloudSurfBuf.pop();
                mutex_lock.unlock();
                continue;              
            }


            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*pointCloudEdgeBuf.front(), *pointcloud_edge_in);
            pcl::fromROSMsg(*pointCloudSurfBuf.front(), *pointcloud_surf_in);
            ros::Time pointcloud_time = (odometryBuf.front())->header.stamp;
            Eigen::Isometry3d odom_in = Eigen::Isometry3d::Identity();
            odom_in.rotate(Eigen::Quaterniond(odometryBuf.front()->pose.pose.orientation.w,odometryBuf.front()->pose.pose.orientation.x,odometryBuf.front()->pose.pose.orientation.y,odometryBuf.front()->pose.pose.orientation.z));  
            odom_in.pretranslate(Eigen::Vector3d(odometryBuf.front()->pose.pose.position.x,odometryBuf.front()->pose.pose.position.y,odometryBuf.front()->pose.pose.position.z));
            current_frame_id = loopInfoBuf.front()->current_id;
            matched_frame_id.clear();
            for(int i=0;i<(int)loopInfoBuf.front()->matched_id.size();i++){
                matched_frame_id.push_back(loopInfoBuf.front()->matched_id[i]);
            }
            pointCloudEdgeBuf.pop();
            pointCloudSurfBuf.pop();
            odometryBuf.pop();
            loopInfoBuf.pop();
            mutex_lock.unlock();

            if(current_frame_id!= (int)iscOptimization.pointcloud_surf_arr.size()){
                ROS_WARN_ONCE("graph optimization frame not aligned,pls check your data");
            }

            //ROS_INFO("total frame size %d",iscOptimization.pointcloud_surf_arr.size());
            if(iscOptimization.addPoseToGraph(pointcloud_edge_in, pointcloud_surf_in,matched_frame_id, odom_in)){
                //update path
                path_optimized.poses.clear();
                //FILE * pFile;
                //pFile = fopen (file_name.c_str(),"w+");
                //ROS_WARN("total frame size %d",iscOptimization.pointcloud_surf_arr.size());
                for(int i=0;i<(int)iscOptimization.pointcloud_surf_arr.size()-1;i++){
                    Eigen::Isometry3d pose = iscOptimization.getPose(i);
                    Eigen::Quaterniond q_temp(pose.rotation());
                    
                    geometry_msgs::PoseStamped pose_temp;
                    pose_temp.pose.position.x = pose.translation().x();
                    pose_temp.pose.position.y = pose.translation().y();
                    pose_temp.pose.position.z = pose.translation().z();
                    pose_temp.pose.orientation.w = q_temp.w();
                    pose_temp.pose.orientation.x = q_temp.x();
                    pose_temp.pose.orientation.y = q_temp.y();
                    pose_temp.pose.orientation.z = q_temp.z();
                    pose_temp.header.stamp = ros::Time::now();
                    pose_temp.header.frame_id = "/map";
                    path_optimized.poses.push_back(pose_temp);


                }
               // fclose (pFile);
            }

            path_optimized.header.seq = (int)iscOptimization.pointcloud_surf_arr.size();
            path_optimized.header.stamp = ros::Time::now();
            Eigen::Isometry3d pose_current = iscOptimization.getLastPose();
            Eigen::Quaterniond q_current(pose_current.rotation());
            Eigen::Vector3d t_current = pose_current.translation();
            w_odom_curr = pose_current * odom_in.inverse();


            geometry_msgs::PoseStamped pose_temp;
            pose_temp.pose.position.x = t_current.x();
            pose_temp.pose.position.y = t_current.y();
            pose_temp.pose.position.z = t_current.z();
            pose_temp.pose.orientation.w = q_current.w();
            pose_temp.pose.orientation.x = q_current.x();
            pose_temp.pose.orientation.y = q_current.y();
            pose_temp.pose.orientation.z = q_current.z();
            pose_temp.header.stamp = ros::Time::now();
            pose_temp.header.frame_id = "/map";
            path_optimized.poses.push_back(pose_temp);
            path_pub.publish(path_optimized);
//TODO_czy
            Eigen::Matrix4d laser_odometry_ = Eigen::Matrix4d::Identity();
            static std::ofstream  laser_odom;
            static bool is_file_created = false;
            if(1)
            {
                laser_odometry_(0,3) = t_current.x();
                laser_odometry_(1,3) = t_current.y();
                laser_odometry_(2,3) = t_current.z();

                Eigen::Quaterniond xuanzhuan_q = Eigen::Quaterniond(q_current.w(),q_current.x(),q_current.y(),q_current.z());


                laser_odometry_.block<3,3>(0,0) = xuanzhuan_q.matrix();

                if (!is_file_created) 
                {
                    is_file_created = true;
                    if (!CreateDirectory(WORK_SPACE_PATH + "/slam_data/trajectory"))
                    {
                        is_file_created = false;
                        std::cout << "CreateDirectory OK! " << std::endl;
                    }

                    if (!CreateFile(laser_odom, WORK_SPACE_PATH + "/slam_data/trajectory/laser_odom.txt"))
                    {
                        is_file_created = false;
                        std::cout << "CreateFile OK! " << std::endl;
                    }
                }

                if(is_file_created)
                {
                    for (int i = 0; i < 3; ++i) 
                    {
                        for (int j = 0; j < 4; ++j) 
                        {
                            laser_odom << laser_odometry_(i, j);
                            
                            if (i == 2 && j == 3) 
                            {
                                laser_odom << std::endl;
                            } 
                            else 
                            {
                                laser_odom << " ";
                            }
                        }
                    }
                }
            }
//TODO_czy
            sensor_msgs::PointCloud2 PointsMsg;
            pcl::toROSMsg(*iscOptimization.loop_map_pc, PointsMsg);
            PointsMsg.header.stamp = pointcloud_time;
            PointsMsg.header.frame_id = "/map";
            loop_map_pub.publish(PointsMsg);

            sensor_msgs::PointCloud2 PointsMsg2;
            pcl::toROSMsg(*iscOptimization.loop_candidate_pc, PointsMsg2);
            PointsMsg2.header.stamp = pointcloud_time;
            PointsMsg2.header.frame_id = "/map";
            loop_candidate_pub.publish(PointsMsg2);
        }
        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }//end of while(1)
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "iscOptimization");
    ros::NodeHandle nh("~");

    ros::Subscriber velodyne_edge_sub= nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_corner", 10, pointCloudEdgeCallback);
    ros::Subscriber velodyne_surf_sub= nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf", 10, pointCloudSurfCallback);
    ros::Subscriber odom_sub= nh.subscribe<nav_msgs::Odometry>("/odom", 10, odomCallback);
    ros::Subscriber loop_sub= nh.subscribe<intensity_slam::LoopInfo>("/loop_closure", 10, loopClosureCallback);

    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom_final", 100);
    path_pub = nh.advertise<nav_msgs::Path>("/final_path", 100);
    loop_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/loop_map", 100);
    loop_candidate_pub = nh.advertise<sensor_msgs::PointCloud2>("/loop_candidate", 100);

    path_optimized.header.frame_id = "/map";   
    path_optimized.header.stamp = ros::Time::now();
    //read parameter
    int sector_width =60;
    int ring_height = 60;
    double max_distance= 40.0;

    int sequence_number =-1;
    nh.getParam("/sector_width", sector_width); 
    nh.getParam("/ring_height", ring_height); 
    nh.getParam("/max_distance", max_distance);  
    nh.getParam("/scan_period", scan_period); 


    //init ISC
    iscOptimization.init();

    //open thread
    std::thread global_optimization_process{global_optimization};

    ros::spin();
    
    return 0;
}
