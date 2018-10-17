/****************************************************************************************
 *   Copyright (C) 2015 - 2018 by                                                       *
 *      Tarek Taha, KURI  <tataha@tarektaha.com>                                        *
 *                                                                                      *
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
#include <geometry_msgs/Pose.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#ifndef UTILS_H_
#define UTILS_H_

Eigen::Matrix4f sensor2RobotTransform(geometry_msgs::Pose location)
{
    tf::Quaternion qt;
    Eigen::Matrix4f sensorPose;
    Eigen::Matrix3d Rd;
    Eigen::Matrix3f Rf;
    sensorPose.setZero();
    qt.setX(location.orientation.x);
    qt.setY(location.orientation.y);
    qt.setZ(location.orientation.z);
    qt.setW(location.orientation.w);
    tf::Matrix3x3 R_tf(qt);
    tf::matrixTFToEigen(R_tf,Rd);
    Rf = Rd.cast<float>();
    sensorPose.block (0, 0, 3, 3) = Rf;
    Eigen::Vector3f T;
    T(0) = location.position.x; T(1) = location.position.y; T(2) = location.position.z;
    sensorPose.block(0, 3, 3, 1) = T;
    sensorPose(3, 3) = 1;
    return sensorPose;
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

visualization_msgs::Marker drawLines(std::vector<geometry_msgs::Point> links, int id, int c_color[], std::string frameId, float scale = 0.08)
{
    visualization_msgs::Marker linksMarkerMsg;
    linksMarkerMsg.header.frame_id = frameId;
    linksMarkerMsg.header.stamp    = ros::Time::now();
    linksMarkerMsg.ns              = "link_marker";
    linksMarkerMsg.id              = id;
    linksMarkerMsg.type            = visualization_msgs::Marker::LINE_LIST;
    linksMarkerMsg.scale.x         = scale;
    linksMarkerMsg.action          = visualization_msgs::Marker::ADD;
    linksMarkerMsg.lifetime        = ros::Duration(1000);

    std_msgs::ColorRGBA color;
    color.r = (float)c_color[0]; color.g=(float)c_color[1]; color.b=(float)c_color[2], color.a=1.0f;

    std::vector<geometry_msgs::Point>::iterator linksIterator;
    for(linksIterator = links.begin(); linksIterator != links.end(); linksIterator++)
    {
        linksMarkerMsg.points.push_back(*linksIterator);
        linksMarkerMsg.colors.push_back(color);
    }
    return linksMarkerMsg;
}

visualization_msgs::Marker drawLines(std::vector<geometry_msgs::Point> links, int id, int c_color[], std::string frameId = "world")
{
    return drawLines(links, id, c_color, frameId, 0.08);
}

#endif