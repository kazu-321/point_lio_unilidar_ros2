#pragma once
#include <cmath>
#include <math.h>
#include <csignal>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>


#define MAX_INI_COUNT (100)

class ImuProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();
  
  void Reset();
  void Reset(double start_timestamp, const sensor_msgs::msg::Imu::SharedPtr &lastimu);
  void Process(const MeasureGroup &meas, PointCloudXYZI::Ptr pcl_un_);
  void Set_init(Eigen::Vector3d &tmp_gravity, Eigen::Matrix3d &rot);

  ofstream fout_imu;
 
  int    lidar_type;
  bool   imu_en;
  V3D mean_acc, gravity_;
  bool   imu_need_init_ = true;
  bool   b_first_frame_ = true;
  bool   gravity_align_ = false;

 private:
  void IMU_init(const MeasureGroup &meas, int &N);
  V3D mean_gyr;
  int    init_iter_num = 1;
};
