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
#include <pcl/filters/voxel_grid.h>
#include <culling/occlusion_culling.h>
#include <culling/utils.h>

// octomap
#include <octomap_msgs/Octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>

#include <string>
using namespace std;
using namespace octomap;

typedef pcl::PointXYZRGB pointType;
//typedef pcl::PointXYZ pointType;
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

    std::string pcdFileName,viewpointsFile, frameId;
    double sensor_roll, sensor_pitch, sensor_yaw, sensor_x, sensor_y, sensor_z;

    // Load Params
    nh.param<std::string>("pcd_input_file", pcdFileName, std::string("sphere_verydensed.pcd"));
    nh.param<std::string>("viewpoints_file", viewpointsFile, std::string("viewpoints.txt"));
    nh.param<std::string>("frame_id", frameId, std::string("world"));
    nh.param<double>("sensor_roll", sensor_roll, 0.0);
    nh.param<double>("sensor_pitch", sensor_pitch, 0.0);
    nh.param<double>("sensor_yaw", sensor_yaw, 0.0);
    nh.param<double>("sensor_x", sensor_x, 0.0);
    nh.param<double>("sensor_y", sensor_y, 0.0);
    nh.param<double>("sensor_z", sensor_z, 0.0);

    std::vector<double> sensorRPY2Robot;
    sensorRPY2Robot.push_back(sensor_roll);
    sensorRPY2Robot.push_back(sensor_pitch);
    sensorRPY2Robot.push_back(sensor_yaw);

    std::vector<double> sensorXYZ2Robot;
    sensorXYZ2Robot.push_back(sensor_x);
    sensorXYZ2Robot.push_back(sensor_y);
    sensorXYZ2Robot.push_back(sensor_z);

    std::string path = ros::package::getPath("culling");
    std::string pcdFilePath = path + "/data/pcd/" + pcdFileName;

    pcl::io::loadPCDFile<pointType> (pcdFilePath, *originalCloud);

    // viewpoints test file
    std::string viewpointsFileStr = path + "/data/" + viewpointsFile;

    double locationx,locationy,locationz,yaw;
    geometry_msgs::PoseArray viewpoints;
    geometry_msgs::PoseStamped robotLocation;
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
        ROS_INFO("Robot location x,y,z:%f,%f,%f ", locationx,locationy,locationz);

        robotLocation.pose.position.x = locationx;
        robotLocation.pose.position.y = locationy;
        robotLocation.pose.position.z = locationz;
        tf::Quaternion tf_q ;
        tf_q = tf::createQuaternionFromYaw(yaw);
        robotLocation.pose.orientation.x = tf_q.getX();
        robotLocation.pose.orientation.y = tf_q.getY();
        robotLocation.pose.orientation.z = tf_q.getZ();
        robotLocation.pose.orientation.w = tf_q.getW();
        robotLocation.header.frame_id    = frameId;
        currentPosePub.publish(robotLocation);

        geometry_msgs::Pose sensorViewPointPose = uav2camTransformation(robotLocation.pose, sensorRPY2Robot, sensorXYZ2Robot);
        viewpoints.poses.push_back(sensorViewPointPose);

        PointCloud tempCloud;
        ros::Time tic = ros::Time::now();

        ///////////////////////////////////////////////////////////////
        tempCloud    = occlusionCulling.extractVisibleSurface(sensorViewPointPose);
        frustumCloud = occlusionCulling.getFrustumCloud();
        ////////////////////////////////////////////////////////////////

        ros::Time toc = ros::Time::now();
        double elapsed =  toc.toSec() - tic.toSec();
        timeSum +=elapsed;
        ROS_INFO("Occulision Culling duration (s) = %f",elapsed);
        occludedCloud += tempCloud;
        frustumCloud  += frustumCloud;
        viewPointCount++;
        break;
    }
    ROS_INFO("On Average Occulision Culling takes (s) =%f",timeSum/viewPointCount);
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
            p.y = viewpoints.poses.at(i).position.y;
            p.z = viewpoints.poses.at(i).position.z;
            lineSegments.push_back(p);

            p.x = viewpoints.poses.at(i+1).position.x;
            p.y = viewpoints.poses.at(i+1).position.y;
            p.z = viewpoints.poses.at(i+1).position.z;
            lineSegments.push_back(p);
        }
    }
    int c_color[3];
    c_color[0]=1; c_color[1]=0; c_color[2]=1;
    int id=0;
    visualization_msgs::Marker linesList = drawLines(lineSegments, id, c_color, frameId, 0.15);


    // *****************Rviz Visualization ************
    ros::Rate loop_rate(1);
    sensor_msgs::PointCloud2 cloud1;
    sensor_msgs::PointCloud2 cloud2;
    sensor_msgs::PointCloud2 cloud5;
    
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