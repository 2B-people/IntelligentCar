/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "chassis.h"
#include "../roborts_sdk/sdk.h"

namespace roborts_base
{
Chassis::Chassis(std::shared_ptr<roborts_sdk::Handle> handle) : handle_(handle)
{
  SDK_Init();
  ROS_Init();
}
void Chassis::SDK_Init()
{
  handle_->CreateSubscriber<roborts_sdk::cmd_imu_data>(CHASSIS_CMD_SET, CMD_PUSH_IMU_DATA,
                                                       CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
                                                       std::bind(&Chassis::ImuInfoCallback, this, std::placeholders::_1));
  // handle_->CreateSubscriber<roborts_sdk::cmd_uwb_info>(COMPATIBLE_CMD_SET, CMD_PUSH_UWB_INFO,
  //                                                      CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
  //                                                      std::bind(&Chassis::UWBInfoCallback, this, std::placeholders::_1));

  chassis_speed_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_chassis_speed>(CHASSIS_CMD_SET, CMD_SET_CHASSIS_SPEED,
                                                                                MANIFOLD2_ADDRESS, CHASSIS_ADDRESS);
  chassis_spd_acc_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_chassis_spd_acc>(CHASSIS_CMD_SET, CMD_SET_CHASSIS_SPD_ACC,
                                                                                    MANIFOLD2_ADDRESS, CHASSIS_ADDRESS);
}
void Chassis::ROS_Init()
{
  //ros publisher
  // ros_odom_pub_ = ros_nh_.advertise<nav_msgs::Odometry>("odom", 30);
  ros_imu_pub_ = ros_nh_.advertise<sensor_msgs::Imu>("/encoder", 1);
  //ros subscriber
  ros_sub_cmd_chassis_vel_ = ros_nh_.subscribe("/car/cmd_vel", 1, &Chassis::ChassisSpeedCtrlCallback, this);
  ros_sub_cmd_chassis_vel_acc_ = ros_nh_.subscribe("cmd_vel_acc", 1, &Chassis::ChassisSpeedAccCtrlCallback, this);

  //ros_message_init
  // imu_data_.header.frame_id = "IMU_link";
  // odom_.header.frame_id = "odom";
  // odom_.child_frame_id = "base_link";

  // odom_tf_.header.frame_id = "odom";
  // odom_tf_.child_frame_id = "base_link";

  // uwb_data_.header.frame_id = "uwb";
}
void Chassis::ImuInfoCallback(const std::shared_ptr<roborts_sdk::cmd_imu_data> imu_info)
{
  // Eigen::Vector3d ea0((-0 - imu_info->ph / 1000) * M_PI / 180.0,
  //                     0,
  //                     0);
  // Eigen::Matrix3d R;
  // R = Eigen::AngleAxisd(ea0[0], ::Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(ea0[1], ::Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(ea0[2], ::Eigen::Vector3d::UnitX());
  // Eigen::Quaterniond q;
  // q = R;

  // imu_data_.orientation.w = (double)q.w();
  // imu_data_.orientation.x = (double)q.x();
  // imu_data_.orientation.y = (double)q.y();
  // imu_data_.orientation.z = (double)q.z();

  // imu_data_.header.stamp = ros::Time::now();
  // imu_data_.angular_velocity.x = (double)imu_info->aax / 1000;
  // imu_data_.angular_velocity.y = (double)imu_info->aay / 1000;
  // imu_data_.angular_velocity.z = (double)imu_info->aaz / 1000;
  // imu_data_.linear_acceleration.x = imu_info->ax * 9.81 / 1000;
  // imu_data_.linear_acceleration.y = imu_info->ay * 9.81 / 1000;
  // imu_data_.linear_acceleration.z = imu_info->az * 9.81 / 1000;
  // ros_imu_pub_.publish(imu_data_);
}

void Chassis::ChassisSpeedCtrlCallback(const geometry_msgs::Twist::ConstPtr &vel)
{
  roborts_sdk::cmd_chassis_speed chassis_speed;
  chassis_speed.vx = vel->linear.x;
  chassis_speed.vy = 0.0;
  chassis_speed.vw = vel->angular.z;
  chassis_speed.rotate_x_offset = 0;
  chassis_speed.rotate_y_offset = 0;
  chassis_speed_pub_->Publish(chassis_speed);
}

void Chassis::ChassisSpeedAccCtrlCallback(const roborts_msgs::TwistAccel::ConstPtr &vel_acc)
{
  roborts_sdk::cmd_chassis_spd_acc chassis_spd_acc;
  chassis_spd_acc.vx = vel_acc->twist.linear.x * 1000;
  chassis_spd_acc.vy = 0.0;
  chassis_spd_acc.vw = vel_acc->twist.angular.z * 1000;
  chassis_spd_acc.ax = vel_acc->accel.linear.x * 1000;
  chassis_spd_acc.ay = 0.0;
  chassis_spd_acc.wz = vel_acc->accel.angular.z * 1000;
  chassis_spd_acc.rotate_x_offset = 0;
  chassis_spd_acc.rotate_y_offset = 0;
  chassis_spd_acc_pub_->Publish(chassis_spd_acc);
}
} // namespace roborts_base
