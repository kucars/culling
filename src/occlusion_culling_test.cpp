#include "ros/ros.h"
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/Pose.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <deque>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include <cmath>
//PCL
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image.h>
//#include <voxel_grid_occlusion_estimation.h>
//#include "fcl_utility.h"
#include <pcl/filters/voxel_grid.h>
#include <culling/occlusion_culling.h>

// octomap
#include <octomap_msgs/Octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>

#include <string>
using namespace std;
using namespace octomap;
//using namespace fcl;

visualization_msgs::Marker drawLines(std::vector<geometry_msgs::Point> links, int c_color, double scale);
geometry_msgs::Pose uav2camTransformation(geometry_msgs::Pose pose, std::vector<double> rpy, std::vector<double> xyz);

//typedef pcl::PointXYZRGB pointType;
typedef pcl::PointXYZ pointType;
typedef pcl::PointCloud<pointType> PointCloud;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "occlusion_culling_test");
    ros::NodeHandle nh;

    ros::Publisher originalCloudPub             = nh.advertise<sensor_msgs::PointCloud2>("original_pointcloud", 100);
    ros::Publisher occludedCloudPub             = nh.advertise<sensor_msgs::PointCloud2>("occluded_pointcloud", 100);
    ros::Publisher frustumCloudPub              = nh.advertise<sensor_msgs::PointCloud2>("frustum_pointcloud", 100);
    ros::Publisher currentPosePub               = nh.advertise<geometry_msgs::PoseStamped>("currentPose", 100);
    ros::Publisher sensorPosePub                = nh.advertise<geometry_msgs::PoseArray>("sensor_pose", 10);
    ros::Publisher generagedPathPub             = nh.advertise<visualization_msgs::Marker>("generated_path", 10);
    ros::Publisher sensorFovPub                 = nh.advertise<visualization_msgs::MarkerArray>("sensor_fov", 100);
    PointCloud occludedCloud;
    PointCloud frustumCloud;

    PointCloud::Ptr originalCloud(new PointCloud);
    PointCloud::Ptr occludedCloudPtr(new PointCloud);
    PointCloud::Ptr frustumCloudPtr(new PointCloud);

    std::string pcdFileName,viewpointsFile;
    double sensor_roll, sensor_pitch, sensor_yaw, sensor_x, sensor_y, sensor_z;
    // Load Params
    nh.param<std::string>("pcd_input_file", pcdFileName, std::string("sphere_verydensed.pcd"));
    nh.param<std::string>("viewpoints_file", viewpointsFile, std::string("viewpoints.txt"));

    nh.param<double>("sensor_roll", sensor_roll, 0.0);
    nh.param<double>("sensor_pitch", sensor_pitch, 0.0);
    nh.param<double>("sensor_yaw", sensor_yaw, 0.0);
    nh.param<double>("sensor_x", sensor_x, 0.0);
    nh.param<double>("sensor_y", sensor_y, 0.0);
    nh.param<double>("sensor_z", sensor_z, 0.0);

    std::vector<double> rpy;
    rpy.push_back(sensor_roll);
    rpy.push_back(sensor_pitch);
    rpy.push_back(sensor_yaw);

    std::vector<double> xyz;
    xyz.push_back(sensor_x);
    xyz.push_back(sensor_y);
    xyz.push_back(sensor_z);

    std::string path = ros::package::getPath("culling");
    std::string pcdFilePath = path + "/data/pcd/" + pcdFileName;

    pcl::io::loadPCDFile<pointType> (pcdFilePath, *originalCloud);

    // viewpoints test file
    std::string viewpointsFileStr = path + "/data/" + viewpointsFile;

    double locationx,locationy,locationz,yaw;
    geometry_msgs::PoseArray viewpoints;
    geometry_msgs::PoseStamped loc;
    double viewPointCount=0;
    double timeSum=0;

    FILE *file = fopen(viewpointsFileStr.c_str(), "r");
    if (!file)
    {
        std::cout<<"\nCan not open the File";
        fclose(file);
    }

    OcclusionCulling<pointType> occlusionCulling(nh,pcdFilePath);
    while (!feof(file))
    {
        fscanf(file,"%lf %lf %lf %lf\n",&locationx,&locationy,&locationz,&yaw);
        std::cout << "location in test: " << locationx << " " << locationy << " " << locationz << std::endl ;
        loc.pose.position.x = locationx;
        loc.pose.position.y = locationy;
        loc.pose.position.z = locationz;
        tf::Quaternion tf_q ;
        tf_q = tf::createQuaternionFromYaw(yaw);
        loc.pose.orientation.x =tf_q.getX();
        loc.pose.orientation.y = tf_q.getY();
        loc.pose.orientation.z = tf_q.getZ();
        loc.pose.orientation.w =tf_q.getW();
        loc.header.frame_id="world";
        currentPosePub.publish(loc);
        viewpoints.poses.push_back(loc.pose);

        geometry_msgs::Pose pt;
        pt.position.x = locationx;
        pt.position.y = locationy;
        pt.position.z = locationz;
        tf::Quaternion tf_q2 ;
        tf_q2 = tf::createQuaternionFromYaw(yaw);
        pt.orientation.x = tf_q2.getX();
        pt.orientation.y = tf_q2.getY();
        pt.orientation.z = tf_q2.getZ();
        pt.orientation.w = tf_q2.getW();
        geometry_msgs::Pose c = uav2camTransformation( pt,  rpy,  xyz);

        ////////////////////////////// Frustum culling - visiualization only //////////////////////////////////////////////////////
        /*
        pcl::FrustumCullingTT fc(true);
        fc.setInputCloud (originalCloud);
        fc.setVerticalFOV (45);
        fc.setHorizontalFOV (58);
        fc.setNearPlaneDistance (0.3);
        fc.setFarPlaneDistance (3.0);
        Eigen::Matrix4f camera_pose;
        Eigen::Matrix3d Rd;
        Eigen::Matrix3f Rf;
        camera_pose.setZero();
        tf::Quaternion qt;
        qt.setX(loc.pose.orientation.x);
        qt.setY(loc.pose.orientation.y);
        qt.setZ(loc.pose.orientation.z);
        qt.setW(loc.pose.orientation.w);
        tf::Matrix3x3 R_tf(qt);
        tf::matrixTFToEigen(R_tf,Rd);
        Rf = Rd.cast<float>();
        camera_pose.block (0, 0, 3, 3) = Rf;
        Eigen::Vector3f T;
        T (0) = loc.pose.position.x; T (1) = loc.pose.position.y; T (2) = loc.pose.position.z;
        camera_pose.block (0, 3, 3, 1) = T;
        camera_pose (3, 3) = 1;
        fc.setCameraPose (camera_pose);
        fc.filter (*FrustumCloud);
        */
        ///////////////////////////////////////////////////////////////////////////
        // 2 *****Occlusion Culling*******


        PointCloud tempCloud;
        ros::Time tic = ros::Time::now();

        ///////////////////////////////////////////////////////////////
        tempCloud = occlusionCulling.extractVisibleSurface(c);
        ////////////////////////////////////////////////////////////////

        ros::Time toc = ros::Time::now();
        double elapsed =  toc.toSec() - tic.toSec();
        timeSum +=elapsed;
        std::cout<<"\nOcculision Culling duration (s) = "<<elapsed<<"\n";
        occludedCloud += tempCloud;
        frustumCloud  += frustumCloud;
        viewPointCount++;
        break;
    }
    std::cout<<"On Average Occulision Culling takes (s) = "<<timeSum/viewPointCount<<"\n";
    occludedCloudPtr->points = occludedCloud.points;
    frustumCloudPtr->points  = frustumCloud.points ;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    // Draw Path
    std::vector<geometry_msgs::Point> lineSegments;
    geometry_msgs::Point p;
    for (uint i =0; i<viewpoints.poses.size(); i++)
    {
        if(i+1< viewpoints.poses.size())
        {
            p.x = viewpoints.poses[i].position.x;
            p.y =  viewpoints.poses.at(i).position.y;
            p.z =  viewpoints.poses.at(i).position.z;
            lineSegments.push_back(p);

            p.x = viewpoints.poses.at(i+1).position.x;
            p.y =  viewpoints.poses.at(i+1).position.y;
            p.z =  viewpoints.poses.at(i+1).position.z;
            lineSegments.push_back(p);
        }
    }
    visualization_msgs::Marker linesList = drawLines(lineSegments,1,0.15);


    // *****************Rviz Visualization ************
    ros::Rate loop_rate(1);
    sensor_msgs::PointCloud2 cloud1;
    sensor_msgs::PointCloud2 cloud2;
    sensor_msgs::PointCloud2 cloud5;
    //occlusionCulling.visualizeOriginalPointcloud() ;
    //***original cloud & occlusion cull publish***
    
    pcl::toROSMsg(*originalCloud, cloud1); //cloud of original
    pcl::toROSMsg(*occludedCloudPtr, cloud2); //cloud of the not occluded voxels (blue) using occlusion culling
    pcl::toROSMsg(*frustumCloudPtr, cloud5); //cloud of the not occluded voxels (blue) using occlusion culling

    while (ros::ok())
    {
        viewpoints.header.frame_id= "world";    
        viewpoints.header.stamp = ros::Time::now();        

        cloud1.header.stamp = ros::Time::now();
        cloud2.header.stamp = ros::Time::now();
        cloud5.header.stamp = ros::Time::now();

        cloud1.header.frame_id = "world";
        cloud2.header.frame_id = "world";
        cloud5.header.frame_id = "world";

        originalCloudPub.publish(cloud1);
        occludedCloudPub.publish(cloud2);
        frustumCloudPub.publish(cloud5);
        sensorPosePub.publish(viewpoints);
        generagedPathPub.publish(linesList);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

visualization_msgs::Marker drawLines(std::vector<geometry_msgs::Point> links, int c_color, double scale)
{
    visualization_msgs::Marker linksMarkerMsg;
    linksMarkerMsg.header.frame_id="world";
    linksMarkerMsg.header.stamp=ros::Time::now();
    linksMarkerMsg.ns="link_marker";
    linksMarkerMsg.id = 0;
    linksMarkerMsg.type = visualization_msgs::Marker::LINE_LIST;
    linksMarkerMsg.scale.x = scale;
    linksMarkerMsg.action  = visualization_msgs::Marker::ADD;
    linksMarkerMsg.lifetime  = ros::Duration(10000.0);
    std_msgs::ColorRGBA color;
    //    color.r = 1.0f; color.g=.0f; color.b=.0f, color.a=1.0f;
    if(c_color == 1)
    {
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;
        color.a = 1.0;
    }
    else if(c_color == 2)
    {
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;
        color.a = 1.0;
    }
    else
    {
        color.r = 0.0;
        color.g = 0.0;
        color.b = 1.0;
        color.a = 1.0;
    }
    std::vector<geometry_msgs::Point>::iterator linksIterator;
    for(linksIterator = links.begin();linksIterator != links.end();linksIterator++)
    {
        linksMarkerMsg.points.push_back(*linksIterator);
        linksMarkerMsg.colors.push_back(color);
    }
    return linksMarkerMsg;
}

geometry_msgs::Pose uav2camTransformation(geometry_msgs::Pose pose, std::vector<double> rpy, std::vector<double> xyz)
{
    Eigen::Matrix4d uav_pose, uav2cam, cam_pose;
    //UAV matrix pose
    Eigen::Matrix3d R; Eigen::Vector3d T1(pose.position.x,pose.position.y,pose.position.z);
    tf::Quaternion qt(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
    tf::Matrix3x3 R1(qt);
    tf::matrixTFToEigen(R1,R);
    uav_pose.setZero ();
    uav_pose.block (0, 0, 3, 3) = R;
    uav_pose.block (0, 3, 3, 1) = T1;
    uav_pose (3, 3) = 1;

    //transformation matrix
    qt = tf::createQuaternionFromRPY(rpy[0],rpy[1],rpy[2]);
    tf::Matrix3x3 R2(qt);Eigen::Vector3d T2(xyz[0],xyz[1],xyz[2]);
    tf::matrixTFToEigen(R2,R);
    uav2cam.setZero ();
    uav2cam.block (0, 0, 3, 3) = R;
    uav2cam.block (0, 3, 3, 1) = T2;
    uav2cam (3, 3) = 1;

    //preform the transformation
    cam_pose = uav_pose * uav2cam;

    Eigen::Matrix4d cam2cam;
    //the transofrmation is rotation by +90 around x axis of the camera
    cam2cam <<   1, 0, 0, 0,
            0, 0,-1, 0,
            0, 1, 0, 0,
            0, 0, 0, 1;
    Eigen::Matrix4d cam_pose_new = cam_pose * cam2cam;
    geometry_msgs::Pose p;
    Eigen::Vector3d T3;Eigen::Matrix3d Rd; tf::Matrix3x3 R3;
    Rd = cam_pose_new.block (0, 0, 3, 3);
    tf::matrixEigenToTF(Rd,R3);
    T3 = cam_pose_new.block (0, 3, 3, 1);
    p.position.x=T3[0];p.position.y=T3[1];p.position.z=T3[2];
    R3.getRotation(qt);
    p.orientation.x = qt.getX(); p.orientation.y = qt.getY();p.orientation.z = qt.getZ();p.orientation.w = qt.getW();
    return p;
}
