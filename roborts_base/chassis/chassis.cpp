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

}
void Chassis::ImuInfoCallback(const std::shared_ptr<roborts_sdk::cmd_imu_data> imu_info)
{

  imu_data_.header.stamp = ros::Time::now();
  imu_data_.angular_velocity.x = 0.0;
  imu_data_.angular_velocity.y = 0.0;
  imu_data_.angular_velocity.z = 0.0;
  imu_data_.linear_acceleration.x = imu_info->ax / 1000.0;
  imu_data_.linear_acceleration.y = 0.0;
  imu_data_.linear_acceleration.z = 0.0;
  ros_imu_pub_.publish(imu_data_);
}

void Chassis::ChassisSpeedCtrlCallback(const geometry_msgs::Twist::ConstPtr &vel)
{

  roborts_sdk::cmd_chassis_speed chassis_speed;
  chassis_speed.vx = vel->linear.x;
  chassis_speed.vy = 0.0;
  // 2500.0 - twist.angular.z * 2000.0 / 180.0;
  chassis_speed.vw = 2500.0 - vel->angular.z * 2000.0 / 180.0;
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

/************************************************************************************************************/
/*******************************************RACECAR**********************************************************/

Car::Car(std::shared_ptr<roborts_sdk::Handle> handle) : handle_(handle)
{
  SDK_Init();
  ROS_Init();
}
void Car::SDK_Init()
{
  chassis_speed_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_chassis_speed>(CHASSIS_CMD_SET, CMD_SET_CHASSIS_SPEED,
                                                                                MANIFOLD2_ADDRESS, CHASSIS_ADDRESS);
  chassis_spd_acc_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_chassis_spd_acc>(CHASSIS_CMD_SET, CMD_SET_CHASSIS_SPD_ACC,
                                                                                    MANIFOLD2_ADDRESS, CHASSIS_ADDRESS);
}
void Car::ROS_Init()
{
  //ros subscriber
  ros_sub_cmd_chassis_vel_ = ros_nh_.subscribe("/car/cmd_vel", 1, &Car::ChassisSpeedCtrlCallback, this);
  ros_sub_cmd_chassis_vel_acc_ = ros_nh_.subscribe("cmd_vel", 1, &Car::ChassisSpeedAccCtrlCallback, this);

}

void Car::ChassisSpeedCtrlCallback(const geometry_msgs::Twist::ConstPtr &vel)
{

  roborts_sdk::cmd_chassis_speed chassis_speed;
  chassis_speed.vx = vel->linear.x;
  chassis_speed.vy = 0.0;
  // 2500.0 - twist.angular.z * 2000.0 / 180.0;
  chassis_speed.vw = 2500.0 - vel->angular.z * 2000.0 / 180.0;
  chassis_speed.rotate_x_offset = 0;
  chassis_speed.rotate_y_offset = 0;
  chassis_speed_pub_->Publish(chassis_speed);
}

void Car::ChassisSpeedAccCtrlCallback(const geometry_msgs::Twist::ConstPtr &vel_acc)
{

  roborts_sdk::cmd_chassis_spd_acc chassis_spd_acc;
  chassis_spd_acc.vx = vel_acc->linear.x * 1000;
  chassis_spd_acc.vy = 0.0;
  chassis_spd_acc.vw = vel_acc->angular.z * 1000;
  chassis_spd_acc.ax = vel_acc->linear.x * 1000;
  chassis_spd_acc.ay = 0.0;
  chassis_spd_acc.wz = vel_acc->angular.z * 1000;
  chassis_spd_acc.rotate_x_offset = 0;
  chassis_spd_acc.rotate_y_offset = 0;
  chassis_spd_acc_pub_->Publish(chassis_spd_acc);
}

} // namespace roborts_base
