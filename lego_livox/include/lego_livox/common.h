// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               Livox@gmail.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include "cloud_msgs/cloud_info.h"

#include <opencv/cv.h>

#include <eigen3/Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

#include <chrono>
// #include <ceres/ceres.h>
// #include <ceres/rotation.h>

#define PI 3.14159265

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

inline double rad2deg(double radians) { return radians * 180.0 / M_PI; }

inline double deg2rad(double degrees) { return degrees * M_PI / 180.0; }

extern const bool loopClosureEnableFlag = true;
extern const double mappingProcessInterval = 0.3;

// Livox
extern const int N_SCAN = 6;
extern const int Horizon_SCAN = 4000;
extern const float ang_res_x = 0.2;
extern const float ang_res_y = 2.0;
extern const float ang_bottom = 15.0+0.1;
extern const int groundScanInd = 7;

extern const float livox_height = 0.5;
extern const int num_lpr = 20;
extern const float th_seeds = 0.5;
extern const int num_iter = 3;
extern const float th_dist = 0.2;

extern const float DISTANCE_SQ_THRESHOLD = 3.0;

extern const float ext_livox[] = {0.0, 0.0, 0.0, 0.0, 0.07, 0.0};

extern const float scanPeriod = 0.1;
extern const int systemDelay = 0;
extern const int imuQueLength = 200;
extern const std::string imuTopic = "/imu/data";

extern const float sensorMountAngle = 0.0;
extern const float segmentTheta = 1.0472; // segmentTheta=1.0472<==>60度,在imageProjection中用于判断平面
extern const int segmentValidPointNum = 5;
extern const int segmentValidLineNum = 3;
extern const float segmentAlphaX = ang_res_x / 180.0 * M_PI;
extern const float segmentAlphaY = ang_res_y / 180.0 * M_PI;


extern const int edgeFeatureNum = 2;
extern const int surfFeatureNum = 4;
extern const int sectionsTotal = 6;
extern const float edgeThreshold = 0.9;
extern const float surfThreshold = 0.9;
extern const float nearestFeatureSearchSqDist = 25;

extern const float surroundingKeyframeSearchRadius = 50.0;
extern const int   surroundingKeyframeSearchNum = 50;

void anti_symmetric(Eigen::Vector3d const &_v, Eigen::Matrix3d &_m)
{
    _m(0, 0) = 0.0;
    _m(0, 1) = -_v.z();
    _m(0, 2) = _v.y();
    _m(1, 0) = _v.z();
    _m(1, 1) = 0.0;
    _m(1, 2) = -_v.x();
    _m(2, 0) = -_v.y();
    _m(2, 1) = _v.x();
    _m(2, 2) = 0.0;
}

struct smoothness_t{ 
    float value;
    size_t ind;
};

struct point_height{ 
    float value;
    size_t ind;
};

struct by_height{ 
    bool operator()(point_height const &left, point_height const &right) { 
        return left.value < right.value;
    }
};

struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;
    }
};

struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time)
)

typedef PointXYZIRPYT  PointTypePose;

// struct LidarEdgeFactor {
//   LidarEdgeFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_,
//                   Eigen::Vector3d last_point_b_, double s_)
//       : curr_point(curr_point_),
//         last_point_a(last_point_a_),
//         last_point_b(last_point_b_),
//         s(s_) {}

//   template <typename T>
//   bool operator()(const T *q, const T *t, T *residual) const {
//     Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()),
//                               T(curr_point.z())};
//     Eigen::Matrix<T, 3, 1> lpa{T(last_point_a.x()), T(last_point_a.y()),
//                                T(last_point_a.z())};
//     Eigen::Matrix<T, 3, 1> lpb{T(last_point_b.x()), T(last_point_b.y()),
//                                T(last_point_b.z())};

//     // Eigen::Quaternion<T> q_last_curr{q[3], T(s) * q[0], T(s) * q[1], T(s) *
//     // q[2]};
//     Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
//     Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
//     q_last_curr = q_identity.slerp(T(s), q_last_curr);
//     Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2]};

//     Eigen::Matrix<T, 3, 1> lp;
//     lp = q_last_curr * cp + t_last_curr;

//     Eigen::Matrix<T, 3, 1> nu = (lp - lpa).cross(lp - lpb);
//     Eigen::Matrix<T, 3, 1> de = lpa - lpb;

//     residual[0] = nu.x() / de.norm();
//     residual[1] = nu.y() / de.norm();
//     residual[2] = nu.z() / de.norm();

//     return true;
//   }

//   static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_,
//                                      const Eigen::Vector3d last_point_a_,
//                                      const Eigen::Vector3d last_point_b_,
//                                      const double s_) {
//     return (new ceres::AutoDiffCostFunction<LidarEdgeFactor, 3, 4, 3>(
//         new LidarEdgeFactor(curr_point_, last_point_a_, last_point_b_, s_)));
//   }

//   Eigen::Vector3d curr_point, last_point_a, last_point_b;
//   double s;
// };