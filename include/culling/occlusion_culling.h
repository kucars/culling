/***************************************************************************
 *   Copyright (C) 2015 - 2017 by                                          *
 *      Tarek Taha, KURI  <tataha@tarektaha.com>                           *
 *      Randa Almadhoun   <randa.almadhoun@kustar.ac.ae>                   *
 *                                                                         *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Steet, Fifth Floor, Boston, MA  02111-1307, USA.          *
 ***************************************************************************/
#ifndef OCCLUSION_H_
#define OCCLUSION_H_

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "ros/ros.h"

//PCL
#include <culling/frustum_culling.h>
#include <culling/voxel_grid_occlusion_estimation.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseArray.h>
#include <pcl/common/eigen.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include "utils.h"

template <typename PointInT>
class OcclusionCulling
{
  public:
    ros::NodeHandle nh;
    std::string model, frameId;

    typename pcl::PointCloud<PointInT>::Ptr cloud;
    typename pcl::PointCloud<PointInT>::Ptr cloudCopy;
    typename pcl::PointCloud<PointInT>::Ptr filteredCloud;
    typename pcl::PointCloud<PointInT>::Ptr occlusionFreeCloud;
    typename pcl::PointCloud<PointInT>::Ptr frustumCloud;
    typename pcl::PointCloud<PointInT>::Ptr rayCloud;
    typename pcl::PointCloud<PointInT>::Ptr occupancyGrid;

    pcl::PointCloud<PointInT> freeCloud;
    double voxelRes, originalVoxelsSize;
    double sensorHorFOV, sensorVerFOV, sensorNearLimit, sensorFarLimit, leafSize;
    double id;
    pcl::VoxelGridOcclusionEstimationT<PointInT> voxelFilterOriginal;
    Eigen::Vector3i max_b1, min_b1;
    pcl::FrustumCullingTT<PointInT> fc;
    double maxAccuracyError, minAccuracyError;
    bool AccuracyMaxSet;
    bool debugEnabled;
    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker rayLines;
    sensor_msgs::PointCloud2 occupancyGridCloud;
    //methods
    OcclusionCulling(ros::NodeHandle& n, std::string modelName);
    OcclusionCulling(ros::NodeHandle& n, typename pcl::PointCloud<PointInT>::Ptr& cloudPtr);
    ~OcclusionCulling();

    pcl::PointCloud<PointInT> extractVisibleSurface(geometry_msgs::Pose location);
    pcl::PointCloud<PointInT> getFrustumCloud();
    visualization_msgs::MarkerArray getFOV();
    sensor_msgs::PointCloud2 getOccupancyGridCloud();
    visualization_msgs::Marker getRays();
    //    float calcCoveragePercent(geometry_msgs::Pose location);
    void initialize();
    float calcCoveragePercent(typename pcl::PointCloud<PointInT>::Ptr cloud_filtered);
    double calcAvgAccuracy(pcl::PointCloud<PointInT> pointCloud);
    double calcAvgAccuracy(pcl::PointCloud<PointInT> pointCloud, geometry_msgs::Pose cameraPose);
    void transformPointMatVec(tf::Vector3 translation, tf::Matrix3x3 rotation,
                              geometry_msgs::Point32 in, geometry_msgs::Point32& out);
    pcl::PointCloud<PointInT> pointCloudViewportTransform(pcl::PointCloud<PointInT> pointCloud,
                                                          geometry_msgs::Pose cameraPose);
    void SSMaxMinAccuracy(std::vector<geometry_msgs::PoseArray> sensorsPoses);
    void visualizeFOV(geometry_msgs::Pose location);
    void visualizeRaycast(geometry_msgs::Pose location,
                          std::vector<geometry_msgs::Point> lineSegments);
    bool contains(pcl::PointCloud<PointInT> c, PointInT p);
    pcl::PointCloud<PointInT> pointsDifference(pcl::PointCloud<PointInT> c2);
};
#include "culling/occlusion_culling.hpp"
#endif
