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

#include "culling/occlusion_culling.h"

template<typename PointInT>
OcclusionCulling<PointInT>::OcclusionCulling(ros::NodeHandle &n, std::string modelName):
    nh(n),
    model(modelName),
    fc(true)
{
    ROS_INFO("Occlusion culling constructor ");
    rayCloud =  pcl::PointCloud<PointInT>::Ptr(new pcl::PointCloud<PointInT>);
    cloud = pcl::PointCloud<PointInT>::Ptr(new pcl::PointCloud <PointInT>); // cloud of original model
    //occlusionFreeCloud = pcl::PointCloud<PointInT>::Ptr(new pcl::PointCloud <PointInT>);
    //FrustumCloud = pcl::PointCloud<PointInT>::Ptr(new pcl::PointCloud <PointInT>);
    std::string path = ros::package::getPath("usar_exploration");
    pcl::io::loadPCDFile<PointInT> (path+"/resources/pcd/"+model, *cloud);
    voxelRes = 0.5;
    OriginalVoxelsSize=0.0;
    id=0.0;
    fc.setInputCloud (cloud);
    fc.setVerticalFOV (45);
    fc.setHorizontalFOV (58);
    fc.setNearPlaneDistance (0.3);
    fc.setFarPlaneDistance (3.0);
}
/*
template<typename PointInT>
OcclusionCulling<PointInT>::OcclusionCulling(ros::NodeHandle &n, typename pcl::PointCloud<PointInT>::Ptr& cloudPtr):
    nh(n),
    fc(true)
{
//   original_pub = nh.advertise<sensor_msgs::PointCloud2>("original_point_cloud", 10);
//   visible_pub = nh.advertise<sensor_msgs::PointCloud2>("occlusion_free_cloud", 100);

   fov_pub = nh.advertise<visualization_msgs::MarkerArray>("fov", 100);
   cloud = pcl::PointCloud<PointInT>::Ptr(new pcl::PointCloud <PointInT>);
   cloudCopy = pcl::PointCloud<PointInT>::Ptr(new pcl::PointCloud <PointInT>);
   filtered_cloud = pcl::PointCloud<PointInT>::Ptr(new pcl::PointCloud <PointInT>);

   occlusionFreeCloud = pcl::PointCloud<PointInT>::Ptr(new pcl::PointCloud <PointInT>);
   FrustumCloud = pcl::PointCloud<PointInT>::Ptr(new pcl::PointCloud <PointInT>);
   cloud->points = cloudPtr->points;
   cloudCopy->points = cloud->points;
    
   voxelRes = 0.1;
   frame_id = "world";
   OriginalVoxelsSize=0.0;
   id=0.0;
   voxelFilterOriginal.setInputCloud (cloud);
   voxelFilterOriginal.setLeafSize (voxelRes, voxelRes, voxelRes);
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
                   OriginalVoxelsSize++;
               }

           }
       }
   }

   pcl::VoxelGrid<PointInT> voxelgrid;
   voxelgrid.setInputCloud (cloud);
   voxelgrid.setLeafSize (0.1f, 0.1f, 0.1f);
   voxelgrid.filter (*filtered_cloud);

   fc.setInputCloud (cloud);
   fc.setVerticalFOV (45);
   fc.setHorizontalFOV (60);
   fc.setNearPlaneDistance (0.5);
   fc.setFarPlaneDistance (8.0);

}

template<typename PointInT>
OcclusionCulling<PointInT>::OcclusionCulling(std::string modelName):
    model(modelName),
    fc(true)
{
    cloud = pcl::PointCloud<PointInT>::Ptr(new pcl::PointCloud <PointInT>);
    filtered_cloud = pcl::PointCloud<PointInT>::Ptr(new pcl::PointCloud <PointInT>);
    cloudCopy = pcl::PointCloud<PointInT>::Ptr(new pcl::PointCloud <PointInT>);
    FrustumCloud = pcl::PointCloud<PointInT>::Ptr(new pcl::PointCloud <PointInT>);

//    occlusionFreeCloud = pcl::PointCloud<PointInT>::Ptr(new pcl::PointCloud <PointInT>);
    std::string path = ros::package::getPath("component_test");
    pcl::io::loadPCDFile<PointInT> (path+"/src/pcd/"+model, *cloud);
    cloudCopy->points = cloud->points;
    voxelRes = 0.5;
    OriginalVoxelsSize=0.0;
    id=0.0;
    voxelFilterOriginal.setInputCloud (cloud);
    voxelFilterOriginal.setLeafSize (voxelRes, voxelRes, voxelRes);
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
                    OriginalVoxelsSize++;
                }

            }
        }
    }
    pcl::VoxelGrid<PointInT> voxelgrid;
    voxelgrid.setInputCloud (cloud);
    voxelgrid.setLeafSize (0.5f, 0.5f, 0.5f);
    voxelgrid.filter (*filtered_cloud);

    fc.setInputCloud (cloud);
    fc.setVerticalFOV (45);
    fc.setHorizontalFOV (58);
    fc.setNearPlaneDistance (0.7);
    fc.setFarPlaneDistance (6.0);

}

template<typename PointInT>
OcclusionCulling<PointInT>::OcclusionCulling():
    model(NULL),
    fc(true)
{
    cloud = pcl::PointCloud<PointInT>::Ptr(new pcl::PointCloud <PointInT>);
    filtered_cloud = pcl::PointCloud<PointInT>::Ptr(new pcl::PointCloud <PointInT>);
    cloudCopy = pcl::PointCloud<PointInT>::Ptr(new pcl::PointCloud <PointInT>);    
    FrustumCloud = pcl::PointCloud<PointInT>::Ptr(new pcl::PointCloud <PointInT>);

//    occlusionFreeCloud = pcl::PointCloud<PointInT>::Ptr(new pcl::PointCloud <PointInT>);
    std::string path = ros::package::getPath("component_test");
    pcl::io::loadPCDFile<PointInT> (path+"/src/pcd/scaled_desktop.pcd", *cloud);
    cloudCopy->points = cloud->points;
    voxelRes = 0.5;
    OriginalVoxelsSize=0.0;
    id=0.0;
    voxelFilterOriginal.setInputCloud (cloud);
    voxelFilterOriginal.setLeafSize (voxelRes, voxelRes, voxelRes);
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
                    OriginalVoxelsSize++;
                }

            }
        }
    }
    pcl::VoxelGrid<PointInT> voxelgrid;
    voxelgrid.setInputCloud (cloud);
    voxelgrid.setLeafSize (0.5f, 0.5f, 0.5f);
    voxelgrid.filter (*filtered_cloud);

    fc.setInputCloud (cloud);
    fc.setVerticalFOV (45);
    fc.setHorizontalFOV (58);
    fc.setNearPlaneDistance (0.7);
    fc.setFarPlaneDistance (6.0);
}
*/
template<typename PointInT>
OcclusionCulling<PointInT>::~OcclusionCulling()
{
}

template<typename PointInT>
pcl::PointCloud<PointInT> OcclusionCulling<PointInT>::extractVisibleSurface(geometry_msgs::Pose location)
{
    ROS_INFO ("extractVisibleSurface \n" ) ;
    std::cout << "location: " << location.position.x << " " <<location.position.y << " " <<location.position.z << std::endl << std::flush ;

    //    // 1 *****Frustum Culling*******
    typename pcl::PointCloud<PointInT>::Ptr output (new pcl::PointCloud <PointInT>);
    typename pcl::PointCloud<PointInT>::Ptr occlusionFreeCloud_local(new pcl::PointCloud<PointInT>);

    Eigen::Matrix4f camera_pose;
    Eigen::Matrix3d Rd;
    Eigen::Matrix3f Rf;
    camera_pose.setZero();
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
    ros::Time tic = ros::Time::now();
    fc.filter(*output);
    ros::Time toc = ros::Time::now();
    std::cout<<"\nFrustum Filter took:"<< toc.toSec() - tic.toSec() << std::endl << std::flush;
    ROS_INFO ("frustum size %d \n" , output->size() ) ;
    fflush(stdout);

    //2:****voxel grid occlusion estimation *****
    Eigen::Quaternionf quat(qt.w(),qt.x(),qt.y(),qt.z());
    output->sensor_origin_  = Eigen::Vector4f(T[0],T[1],T[2],0);
    output->sensor_orientation_= quat;
    pcl::VoxelGridOcclusionEstimationT<PointInT> voxelFilter;
    voxelFilter.setInputCloud (output);
    voxelFilter.setLeafSize (0.1f, 0.1f, 0.1f);
    //voxelFilter.setLeafSize (1.0f, 1.0f, 1.0f);
    //voxelFilter.setLeafSize (0.5f, 0.5f, 0.5f);
    tic = ros::Time::now();
    voxelFilter.initializeVoxelGrid();
    toc = ros::Time::now();
    std::cout<<"\nVoxel Filter took:"<< toc.toSec() - tic.toSec() << std::endl << std::flush;

    int state,ret;
    PointInT p1,p2;
    PointInT point;
    std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > out_ray;
    //std::vector<geometry_msgs::Point> lineSegments;
    //geometry_msgs::Point linePoint;

    // iterate over the entire frustum points
    std::cout<<"\nNumber of points:"<<(int)output->points.size()<<"\n";fflush(stdout);
     // estimate the occluded space
    std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > occluded_voxels;
    tic = ros::Time::now();
    voxelFilter.occlusionEstimationAll (occluded_voxels);
    toc = ros::Time::now();
    std::cout<<"\nocclusionEstimationAll took:"<< toc.toSec() - tic.toSec() << std::endl << std::flush;
    
    typedef pcl::PointCloud<PointInT> CloudT;
    typename CloudT::Ptr occ_centroids (new CloudT);
    occ_centroids->width = static_cast<int> (occluded_voxels.size ());
    occ_centroids->height = 1;
    occ_centroids->is_dense = false;
    occ_centroids->points.resize (occluded_voxels.size ());
    for (size_t i = 0; i < occluded_voxels.size (); ++i)
    {
        Eigen::Vector4f xyz = voxelFilter.getCentroidCoordinate (occluded_voxels[i]);
        PointInT point;
        point.x = xyz[0];
        point.y = xyz[1];
        point.z = xyz[2];
        occ_centroids->points[i] = point;
    }
    std::cout<<"\nSize of Occluded points:"<<occ_centroids->points.size();fflush(stdout);
    for ( int i = 0; i < (int)output->points.size(); i ++ )
    {
        PointInT ptest = output->points[i];
        //std::cout << "ptest" << ptest.x << "   ptest"<< (int)ptest.r <<  "   ptest"<< (int)ptest.g <<std::endl;
        Eigen::Vector3i ijk = voxelFilter.getGridCoordinates( ptest.x, ptest.y, ptest.z);
        // process all free voxels
       //int index = voxelFilter.getCentroidIndexAt (ijk);
       if(voxelFilter.getCentroidIndexAt(ijk) == -1 ) {
			// Voxel is out of bounds
            continue;
        }        
        Eigen::Vector4f centroid = voxelFilter.getCentroidCoordinate (ijk);
        point = PointInT(0,244,0);
        point.x = centroid[0];
        point.y = centroid[1];
        point.z = centroid[2];
		Eigen::Vector4f direction = centroid - output->sensor_origin_;
		direction.normalize ();
		
		// Estimate entry point into the voxel grid
		float tmin = voxelFilter.rayBoxIntersection (output->sensor_origin_, direction,p1,p2); //where did this 4-input syntax come from?
		
		if(tmin == -1){
			// ray does not intersect with the bounding box
			continue;
		}
        
		// Calculate coordinate of the boundary of the voxel grid
		Eigen::Vector4f start = output->sensor_origin_ + tmin * direction;
		
		// Determine distance between boundary and target voxel centroid
		Eigen::Vector4f dist_vector = centroid-start;
		float distance = (dist_vector).dot(dist_vector);

		if (distance > voxelRes*1.414){ // voxelRes/sqrt(2)
			// ray does not correspond to this point
			continue;
		}
		
		// Save point
		occlusionFreeCloud_local->points.push_back(ptest);
		//occlusionFreeCloud->points.push_back(ptest);
		
        /*
        if(index!=-1 )
        {
            out_ray.clear();
            ret = voxelFilter.occlusionEstimation(state,out_ray, ijk);
            if(state == 1)
            {
                // WARNING : The following code produce a meomery leakage specialy when using a dense and continous point cloud extraction
                // Only used for debugging 
                //                for(uint j=0; j< out_ray.size(); j++)
                //                {
                //                    Eigen::Vector4f centroid = voxelFilter.getCentroidCoordinate (out_ray[j]);
                //                    PointInT p = PointInT(255,255,0);
                //                    p.x = centroid[0];
                //                    p.y = centroid[1];
                //                    p.z = centroid[2];
                //                    rayCloud->points.push_back(p);
                //                   // std::cout<<"Ray X:"<<p.x<<" y:"<< p.y<<" z:"<< p.z<<"\n";
                //                }
            }
            else
            {
                // ROS_INFO("state %d \n" , state ) ;
                // estimate direction to target voxel
                Eigen::Vector4f direction = centroid - cloud->sensor_origin_;
                direction.normalize ();
                // estimate entry point into the voxel grid
                float tmin = voxelFilter.rayBoxIntersection (cloud->sensor_origin_, direction,p1,p2);
                if(tmin!=-1)
                {
                    occlusionFreeCloud_local->points.push_back(ptest);
                    //occlusionFreeCloud->points.push_back(ptest);
                }
            }
        }
        */
    }
    FreeCloud.points = occ_centroids->points;//occlusionFreeCloud_local->points;
    std::cout<<"\nSize of Free points:"<<FreeCloud.points.size();fflush(stdout);
    return FreeCloud ;
}