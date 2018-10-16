/****************************************************************************************
 *   Copyright (C) 2015 - 2017 by                                                       *
 *      Tarek Taha, KURI  <tataha@tarektaha.com>                                        *
 *      Randa Almadhoun   <randa.almadhoun@kustar.ac.ae>                                *
 *                                                                                      *
 *      The package was modified by Reem Ashour as follows:                         	*
 *      1-  All functions were modified to deal with XYZRGB points instead of XYZ   	*
 *      2-  Parameters were changed to work with environment exploration
 *      3-  Coverage caluation function's were removed                              	*
 *      Reem Ashour <reem.ashour@ku.ac.ae>                                	        *
 *
 *   This program is free software; you can redistribute it and/or modify               *
 *   it under the terms of the GNU General Public License as published by               *
 *   the Free Software Foundation; either version 2 of the License, or                  *
 *   (at your option) any later version.                                                *
 *                                                                                      *
 *   This program is distributed in the hope that it will be useful,                    *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of                     *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                      *
 *   GNU General Public License for more details.                                       *
 *                                                                                      *
 *   You should have received a copy of the GNU General Public License                  *
 *   along with this program; if not, write to the                                      *
 *   Free Software Foundation, Inc.,                                                    *
 *   51 Franklin Steet, Fifth Floor, Boston, MA  02111-1307, USA.                       *
 ****************************************************************************************/

#ifndef OCCLUSION_HPP_
#define OCCLUSION_HPP_

#include "culling/occlusion_culling.h"

template<typename PointInT>
void OcclusionCulling<PointInT>::initialize()
{
   ROS_INFO("Occlusion culling constructor");
   ROS_INFO("Cloud Size:%d",cloud->points.size());

   filteredCloud      = typename pcl::PointCloud<PointInT>::Ptr(new pcl::PointCloud <PointInT>);
   occlusionFreeCloud = typename pcl::PointCloud<PointInT>::Ptr(new pcl::PointCloud <PointInT>);
   frustumCloud       = typename pcl::PointCloud<PointInT>::Ptr(new pcl::PointCloud <PointInT>);   
   rayCloud           = typename pcl::PointCloud<PointInT>::Ptr(new pcl::PointCloud<PointInT>);      
   cloudCopy          = typename pcl::PointCloud<PointInT>::Ptr(new pcl::PointCloud <PointInT>);  
   cloudCopy->points  = cloud->points;

   sensor_fov_pub     = nh.advertise<visualization_msgs::MarkerArray>("sensor_fov", 100);

   nh.param<double>("voxel_res", voxelRes, 0.1);
   nh.param<double>("sensor_hor_fov", sensorHorFOV, 60.0);
   nh.param<double>("sensor_ver_fov", sensorVerFOV, 45.0);
   nh.param<double>("sensor_near_limit", sensorNearLimit, 0.5);
   nh.param<double>("sensor_far_limit", sensorFarLimit, 8.0);
   nh.param<std::string>("frame_id", frameId,"world");

   ROS_INFO("Voxel Res:%f",voxelRes);
   ROS_INFO("Sensor h,v,n,f=%f,%f,%f,%f frame id:%s",sensorHorFOV,sensorVerFOV,sensorNearLimit,sensorFarLimit,frameId);

   originalVoxelsSize=0.0;
   id=0.0;

   voxelFilterOriginal.setInputCloud(cloud);
   voxelFilterOriginal.setLeafSize(voxelRes, voxelRes, voxelRes);
   voxelFilterOriginal.initializeVoxelGrid();
   min_b1 = voxelFilterOriginal.getMinBoxCoordinates ();
   max_b1 = voxelFilterOriginal.getMaxBoxCoordinates ();
   for (int kk = min_b1.z (); kk <= max_b1.z (); ++kk)
   {
       for (int jj = min_b1.y (); jj <= max_b1.y (); ++jj)
       {
           for (int ii = min_b1.x (); ii <= max_b1.x (); ++ii)
           {
               Eigen::Vector3i ijk1 (ii, jj, kk);
               int index1 = voxelFilterOriginal.getCentroidIndexAt (ijk1);
               if(index1!=-1)
               {
                   originalVoxelsSize++;
               }

           }
       }
   }

   pcl::VoxelGrid<PointInT> voxelgrid;
   voxelgrid.setInputCloud(cloud);
   voxelgrid.setLeafSize(voxelRes, voxelRes, voxelRes);
   voxelgrid.filter(*filteredCloud);

   fc.setInputCloud(cloud);
   fc.setVerticalFOV(sensorVerFOV);
   fc.setHorizontalFOV(sensorHorFOV);
   fc.setNearPlaneDistance(sensorNearLimit);
   fc.setFarPlaneDistance(sensorFarLimit);
}

template<typename PointInT>
OcclusionCulling<PointInT>::OcclusionCulling(ros::NodeHandle &n, std::string modelName):
    nh(n),
    model(modelName),
    fc(true)
{
    cloud = typename pcl::PointCloud<PointInT>::Ptr(new pcl::PointCloud <PointInT>);
    pcl::io::loadPCDFile<PointInT>(modelName, *cloud);
    initialize();
}

template<typename PointInT>
OcclusionCulling<PointInT>::OcclusionCulling(ros::NodeHandle &n, typename pcl::PointCloud<PointInT>::Ptr& cloudPtr):
    nh(n),
    fc(true)
{
   cloud         = typename pcl::PointCloud<PointInT>::Ptr(new pcl::PointCloud <PointInT>);
   cloud->points = cloudPtr->points;
   initialize();   
}

template<typename PointInT>
OcclusionCulling<PointInT>::~OcclusionCulling()
{
}

template<typename PointInT>
pcl::PointCloud<PointInT> OcclusionCulling<PointInT>::extractVisibleSurface(geometry_msgs::Pose location)
{
    ROS_INFO("ExtractVisibleSurface \n" );
    ROS_INFO("Location X:%f Y:%f Z:%f",location.position.x,location.position.y,location.position.z);

    //*****Frustum Culling*******
    typename pcl::PointCloud<PointInT>::Ptr output (new pcl::PointCloud <PointInT>);
    typename pcl::PointCloud<PointInT>::Ptr occlusionFreeCloud_local(new pcl::PointCloud<PointInT>);

    Eigen::Matrix4f sensorPose = sensor2RobotTransform(location);

    fc.setCameraPose(sensorPose);
    ros::Time tic = ros::Time::now();
    fc.filter(*output);
    ros::Time toc = ros::Time::now();
    std::cout<<"\nFrustum Filter took:"<< toc.toSec() - tic.toSec() << std::endl << std::flush;
    ROS_INFO ("Cloud input size:%d Frustum size:%d \n",cloud->points.size(), output->size()) ;
    fflush(stdout);

    //2:****voxel grid occlusion estimation *****
    tf::Quaternion qt;
    Eigen::Vector3f T;
    qt.setX(location.orientation.x);
    qt.setY(location.orientation.y);
    qt.setZ(location.orientation.z);
    qt.setW(location.orientation.w);    
    Eigen::Quaternionf quat(qt.w(),qt.x(),qt.y(),qt.z());
    T(0) = location.position.x; T(1) = location.position.y; T(2) = location.position.z;
    output->sensor_origin_  = Eigen::Vector4f(T[0],T[1],T[2],0);
    output->sensor_orientation_= quat;
    pcl::VoxelGridOcclusionEstimationT<PointInT> voxelFilter;
    voxelFilter.setInputCloud(output);
    voxelFilter.setLeafSize(voxelRes, voxelRes, voxelRes);

    tic = ros::Time::now();
    voxelFilter.initializeVoxelGrid();
    toc = ros::Time::now();
    std::cout<<"\nVoxel Filter took:"<< toc.toSec() - tic.toSec() << std::endl << std::flush;

    int state,ret;
    PointInT p1,p2;
    PointInT point;
    std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > out_ray;

    // iterate over the entire frustum points
    std::cout<<"\nNumber of points:"<<(int)output->points.size()<<"\n";fflush(stdout);
     
    for (int i = 0; i < (int)output->points.size(); i++)
    {
        PointInT ptest = output->points[i];
        Eigen::Vector3i ijk = voxelFilter.getGridCoordinates(ptest.x, ptest.y, ptest.z);
        if (voxelFilter.getCentroidIndexAt(ijk) == -1)
        {
            // Voxel is out of bounds
            continue;
        }
		// >>>>>>>>>>>>>>>>>>>>
		// 2.1 Perform occlusion estimation
		// >>>>>>>>>>>>>>>>>>>>
		
		/*
		 * The way this works is it traces a line to the point and finds
		 * the distance to the first occupied voxel in its path (t_min).
		 * It then determines if the distance between the target point and
		 * the collided voxel are close (within voxelRes).
		 * 
		 * If they are, the point is visible. Otherwise, the point is behind
		 * an occluding point
		 * 
		 * 
		 * This is the expected function of the following command:
		 *   ret = voxelFilter.occlusionEstimation( state,out_ray, ijk);
		 * 
		 * However, it sometimes shows occluded points on the edge of the cloud
		 */
		
         // Direction to target voxel        
        Eigen::Vector4f centroid = voxelFilter.getCentroidCoordinate(ijk);
        point = PointInT(0, 244, 0);
        point.x = centroid[0];
        point.y = centroid[1];
        point.z = centroid[2];
        Eigen::Vector4f direction = centroid - output->sensor_origin_;
        direction.normalize();

        // Estimate entry point into the voxel grid
        float tmin = voxelFilter.rayBoxIntersection(output->sensor_origin_, direction, p1, p2); //where did this 4-input syntax come from?

        if (tmin == -1)
        {
            // ray does not intersect with the bounding box
            continue;
        }

        // Calculate coordinate of the boundary of the voxel grid
        Eigen::Vector4f start = output->sensor_origin_ + tmin * direction;

        // Determine distance between boundary and target voxel centroid
        Eigen::Vector4f dist_vector = centroid - start;
        float distance = (dist_vector).dot(dist_vector);

        if (distance > voxelRes * 1.414) // voxelRes/sqrt(2)
        { 
            // ray does not correspond to this point
            continue;
        }
        // Save point
        occlusionFreeCloud_local->points.push_back(ptest);
    }
    freeCloud.points = occlusionFreeCloud_local->points;   
    std::cout<<"\nSize of Free points:"<<freeCloud.points.size();fflush(stdout);
    visualizeFOV(location);
    return freeCloud;
}

template<typename PointInT>
void OcclusionCulling<PointInT>::visualizeFOV(geometry_msgs::Pose location)
{
    /*
    typename pcl::PointCloud<PointInT>::Ptr output(new pcl::PointCloud <PointInT>);
    Eigen::Matrix4f camera_pose;
    Eigen::Matrix3d Rd;
    Eigen::Matrix3f Rf;
    camera_pose.setZero ();

    tf::Quaternion qt;
    qt.setX(location.orientation.x);
    qt.setY(location.orientation.y);
    qt.setZ(location.orientation.z);
    qt.setW(location.orientation.w);
    tf::Matrix3x3 R_tf(qt);
    tf::matrixTFToEigen(R_tf,Rd);
    Rf = Rd.cast<float>();
    camera_pose.block (0, 0, 3, 3) = Rf;
    Eigen::Vector3f T;
    T (0) = location.position.x; T (1) = location.position.y; T (2) = location.position.z;
    camera_pose.block (0, 3, 3, 1) = T;
    camera_pose (3, 3) = 1;
    fc.setCameraPose (camera_pose);
    fc.filter(*output);
    */
    //*** visualization the FOV *****
    std::vector<geometry_msgs::Point> fov_points;
    int c_color[3];
    geometry_msgs::Point point1;
    point1.x=fc.fp_bl[0];point1.y=fc.fp_bl[1];point1.z=fc.fp_bl[2]; fov_points.push_back(point1);//0
    point1.x=fc.fp_br[0];point1.y=fc.fp_br[1];point1.z=fc.fp_br[2]; fov_points.push_back(point1);//1
    point1.x=fc.fp_tr[0];point1.y=fc.fp_tr[1];point1.z=fc.fp_tr[2]; fov_points.push_back(point1);//2
    point1.x=fc.fp_tl[0];point1.y=fc.fp_tl[1];point1.z=fc.fp_tl[2]; fov_points.push_back(point1);//3
    point1.x=fc.np_bl[0];point1.y=fc.np_bl[1];point1.z=fc.np_bl[2]; fov_points.push_back(point1);//4
    point1.x=fc.np_br[0];point1.y=fc.np_br[1];point1.z=fc.np_br[2]; fov_points.push_back(point1);//5
    point1.x=fc.np_tr[0];point1.y=fc.np_tr[1];point1.z=fc.np_tr[2]; fov_points.push_back(point1);//6
    point1.x=fc.np_tl[0];point1.y=fc.np_tl[1];point1.z=fc.np_tl[2]; fov_points.push_back(point1);//7

    std::vector<geometry_msgs::Point> fov_linesNear;
    fov_linesNear.push_back(fov_points[4]);fov_linesNear.push_back(fov_points[5]);
    fov_linesNear.push_back(fov_points[5]);fov_linesNear.push_back(fov_points[6]);
    fov_linesNear.push_back(fov_points[6]);fov_linesNear.push_back(fov_points[7]);
    fov_linesNear.push_back(fov_points[7]);fov_linesNear.push_back(fov_points[4]);
    c_color[0]=1; c_color[1]=0; c_color[2]=1;
    linesList1 = drawLines(fov_linesNear,id++,c_color);//purple

    std::vector<geometry_msgs::Point> fov_linesFar;
    fov_linesFar.push_back(fov_points[0]);fov_linesFar.push_back(fov_points[1]);
    fov_linesFar.push_back(fov_points[1]);fov_linesFar.push_back(fov_points[2]);
    fov_linesFar.push_back(fov_points[2]);fov_linesFar.push_back(fov_points[3]);
    fov_linesFar.push_back(fov_points[3]);fov_linesFar.push_back(fov_points[0]);
    c_color[0]=1; c_color[1]=1; c_color[2]=0;
    linesList2 = drawLines(fov_linesFar,id++,c_color);//yellow


    std::vector<geometry_msgs::Point> fov_linestop;
    fov_linestop.push_back(fov_points[7]);fov_linestop.push_back(fov_points[3]);//top
    fov_linestop.push_back(fov_points[6]);fov_linestop.push_back(fov_points[2]);//top
    c_color[0]=0; c_color[1]=1; c_color[2]=0;
    linesList3 = drawLines(fov_linestop,id++,c_color);//green

    std::vector<geometry_msgs::Point> fov_linesbottom;
    fov_linesbottom.push_back(fov_points[5]);fov_linesbottom.push_back(fov_points[1]);//bottom
    fov_linesbottom.push_back(fov_points[4]);fov_linesbottom.push_back(fov_points[0]);//bottom
    c_color[0]=0; c_color[1]=0; c_color[2]=1;
    linesList4 = drawLines(fov_linesbottom,id++,c_color);//blue

    marker_array.markers.push_back(linesList1);
    marker_array.markers.push_back(linesList2);
    marker_array.markers.push_back(linesList3);
    marker_array.markers.push_back(linesList4);
    sensor_fov_pub.publish(marker_array);
}

template<typename PointInT>
visualization_msgs::Marker OcclusionCulling<PointInT>::drawLines(std::vector<geometry_msgs::Point> links, int id, int c_color[])
{
    visualization_msgs::Marker linksMarkerMsg;
    linksMarkerMsg.header.frame_id=frameId;
    linksMarkerMsg.header.stamp=ros::Time::now();
    linksMarkerMsg.ns="link_marker";
    linksMarkerMsg.id = id;
    linksMarkerMsg.type = visualization_msgs::Marker::LINE_LIST;
    linksMarkerMsg.scale.x = 0.08;
    linksMarkerMsg.action  = visualization_msgs::Marker::ADD;
    linksMarkerMsg.lifetime  = ros::Duration(1000);
    std_msgs::ColorRGBA color;
    color.r = (float)c_color[0]; color.g=(float)c_color[1]; color.b=(float)c_color[2], color.a=1.0f;

    std::vector<geometry_msgs::Point>::iterator linksIterator;
    for(linksIterator = links.begin();linksIterator != links.end();linksIterator++)
    {
        linksMarkerMsg.points.push_back(*linksIterator);
        linksMarkerMsg.colors.push_back(color);
    }
   return linksMarkerMsg;
}

#endif
