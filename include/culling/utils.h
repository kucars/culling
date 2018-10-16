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

#endif