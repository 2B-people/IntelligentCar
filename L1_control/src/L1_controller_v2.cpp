/*
Copyright (c) 2017, ChanYuan KUO, YoRu LU,
latest editor: HaoChih, LIN
All rights reserved. (Hypha ROS Workshop)
This file is part of hypha_racecar package.
hypha_racecar is free software: you can redistribute it and/or modify
it under the terms of the GNU LESSER GENERAL PUBLIC LICENSE as published
by the Free Software Foundation, either version 3 of the License, or
any later version.
hypha_racecar is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU LESSER GENERAL PUBLIC LICENSE for more details.
You should have received a copy of the GNU LESSER GENERAL PUBLIC LICENSE
along with hypha_racecar.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <thread>

#include "ros/ros.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "nav_msgs/Path.h"
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#define PI 3.14159265358979

/********************/
/* CLASS DEFINITION */
/********************/
class L1Controller
{
public:
    L1Controller();
    void initMarker();
    bool isForwardWayPt(const geometry_msgs::Point &wayPt, const geometry_msgs::Pose &carPose);
    bool isWayPtAwayFromLfwDist(const geometry_msgs::Point &wayPt, const geometry_msgs::Point &car_pos);
    double getYawFromPose(const geometry_msgs::Pose &carPose);
    double getEta(const geometry_msgs::Pose &carPose);
    double getCar2GoalDist();
    double getL1Distance(const double &_Vcmd);
    double getSteeringAngle(double eta);
    double getGasInput(const float &current_v);
    void ReStart();
    void GoCar();
    geometry_msgs::Point get_odom_car2WayPtVec(const geometry_msgs::Pose &carPose);

private:
    ros::NodeHandle n_;
    ros::Subscriber odom_sub, path_sub, goal_sub;
    ros::Publisher pub_, marker_pub;
    ros::Timer timer1, timer2;
    tf::TransformListener tf_listener;

    visualization_msgs::Marker points, line_strip, goal_circle;
    geometry_msgs::Twist cmd_vel;
    geometry_msgs::Point odom_goal_pos;
    nav_msgs::Odometry odom;
    nav_msgs::Path map_path, odom_path;

    double L, Lfw, Lrv, Vcmd, lfw, lrv, steering, u, v;
    double Gas_gain, baseAngle, Angle_gain, goalRadius;
    int controller_freq, baseSpeed;
    int now_speed_;
    int start_loop_, loop_;
    double start_speed_;
    bool foundForwardPt, goal_received, goal_reached;
    bool go_;

    void odomCB(const nav_msgs::Odometry::ConstPtr &odomMsg);
    void pathCB(const nav_msgs::Path::ConstPtr &pathMsg);
    void goalCB(const geometry_msgs::PoseStamped::ConstPtr &goalMsg);
    void goalReachingCB(const ros::TimerEvent &);
    void controlLoopCB(const ros::TimerEvent &);

}; // end of class

L1Controller::L1Controller()
{
    //Private parameters handler
    ros::NodeHandle pn("~");

    //no use
    pn.param("Lrv", Lrv, 10.0);
    pn.param("lrv", lrv, 10.0);
    pn.param("GasGain", Gas_gain, 1.0);

    //Car parameter
    pn.param("L", L, 0.26);
    pn.param("Vcmd", Vcmd, 1.0);
    pn.param("lfw", lfw, 0.13);

    //Controller parameter
    pn.param("controller_freq", controller_freq, 20);
    pn.param("AngleGain", Angle_gain, -1.0);
    pn.param("baseSpeed", baseSpeed, 5100);
    pn.param("baseAngle", baseAngle, 90.0);

    //start
    pn.param("startSpeed", start_speed_, 5100.0);
    pn.param("startLoop", start_loop_, 60);
    loop_ = 0;

    //Publishers and Subscribers
    odom_sub = n_.subscribe("/odometry/filtered", 1, &L1Controller::odomCB, this);
    path_sub = n_.subscribe("/move_base_node/NavfnROS/plan", 1, &L1Controller::pathCB, this);
    goal_sub = n_.subscribe("/move_base_simple/goal", 1, &L1Controller::goalCB, this);

    marker_pub = n_.advertise<visualization_msgs::Marker>("car_path", 10);
    pub_ = n_.advertise<geometry_msgs::Twist>("car/cmd_vel", 1);

    //Timer
    timer1 = n_.createTimer(ros::Duration((1.0) / controller_freq), &L1Controller::controlLoopCB, this);  // Duration(0.05) -> 20Hz
    timer2 = n_.createTimer(ros::Duration((0.5) / controller_freq), &L1Controller::goalReachingCB, this); // Duration(0.05) -> 20Hz

    //Init variables
    Lfw = goalRadius = getL1Distance(Vcmd);
    cmd_vel.linear.x = 5000; // 5000 for stop
    now_speed_ = 5000;
    cmd_vel.angular.z = baseAngle;

    foundForwardPt = false;
    goal_received = false;
    goal_reached = false;
    go_ = false;

    //Show info
    ROS_INFO("[param] baseSpeed: %d", baseSpeed);
    ROS_INFO("[param] baseAngle: %f", baseAngle);
    ROS_INFO("[param] AngleGain: %f", Angle_gain);
    ROS_INFO("[param] Vcmd: %f", Vcmd);
    ROS_INFO("[param] Lfw: %f", Lfw);

    //Visualization Marker Settings
    initMarker();
}

void L1Controller::ReStart()
{
    start_speed_ = 5100;
    now_speed_ = 5000;
    loop_ = 0;

    goal_reached = true;
    goal_received = false;
    go_ = false;

    cmd_vel.linear.x = 5000; // 1500 for stop
    cmd_vel.angular.z = baseAngle;
    Lfw = goalRadius = getL1Distance(Vcmd);

    ROS_WARN("L1_controller is restart!");
}

void L1Controller::GoCar()
{
    go_ = true;
}

void L1Controller::initMarker()
{
    points.header.frame_id = line_strip.header.frame_id = goal_circle.header.frame_id = "odom";
    points.ns = line_strip.ns = goal_circle.ns = "Markers";
    points.action = line_strip.action = goal_circle.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = goal_circle.pose.orientation.w = 1.0;
    points.id = 0;
    line_strip.id = 1;
    goal_circle.id = 2;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    goal_circle.type = visualization_msgs::Marker::CYLINDER;
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    //LINE_STRIP markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;

    goal_circle.scale.x = goalRadius;
    goal_circle.scale.y = goalRadius;
    goal_circle.scale.z = 0.1;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    //goal_circle is yellow
    goal_circle.color.r = 1.0;
    goal_circle.color.g = 1.0;
    goal_circle.color.b = 0.0;
    goal_circle.color.a = 0.5;
}

void L1Controller::odomCB(const nav_msgs::Odometry::ConstPtr &odomMsg)
{
    odom = *odomMsg;
}

void L1Controller::pathCB(const nav_msgs::Path::ConstPtr &pathMsg)
{
    map_path = *pathMsg;
}

void L1Controller::goalCB(const geometry_msgs::PoseStamped::ConstPtr &goalMsg)
{
    try
    {
        geometry_msgs::PoseStamped odom_goal;
        tf_listener.transformPose("odom", ros::Time(0), *goalMsg, "map", odom_goal);
        odom_goal_pos = odom_goal.pose.position;

        // Start
        // ros::Duration(2.0).sleep();

        goal_received = true;
        goal_reached = false;

        /*Draw Goal on RVIZ*/
        goal_circle.pose = odom_goal.pose;
        marker_pub.publish(goal_circle);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
}

double L1Controller::getYawFromPose(const geometry_msgs::Pose &carPose)
{
    float x = carPose.orientation.x;
    float y = carPose.orientation.y;
    float z = carPose.orientation.z;
    float w = carPose.orientation.w;

    double tmp, yaw;
    tf::Quaternion q(x, y, z, w);
    tf::Matrix3x3 quaternion(q);
    quaternion.getRPY(tmp, tmp, yaw);

    return yaw;
}

bool L1Controller::isForwardWayPt(const geometry_msgs::Point &wayPt, const geometry_msgs::Pose &carPose)
{
    float car2wayPt_x = wayPt.x - carPose.position.x;
    float car2wayPt_y = wayPt.y - carPose.position.y;
    double car_theta = getYawFromPose(carPose);

    float car_car2wayPt_x = cos(car_theta) * car2wayPt_x + sin(car_theta) * car2wayPt_y;
    float car_car2wayPt_y = -sin(car_theta) * car2wayPt_x + cos(car_theta) * car2wayPt_y;

    if (car_car2wayPt_x > 0) /*is Forward WayPt*/
        return true;
    else
        return false;
}

bool L1Controller::isWayPtAwayFromLfwDist(const geometry_msgs::Point &wayPt, const geometry_msgs::Point &car_pos)
{
    double dx = wayPt.x - car_pos.x;
    double dy = wayPt.y - car_pos.y;
    double dist = sqrt(dx * dx + dy * dy);

    if (dist < Lfw)
        return false;
    else if (dist >= Lfw)
        return true;
}

geometry_msgs::Point L1Controller::get_odom_car2WayPtVec(const geometry_msgs::Pose &carPose)
{
    geometry_msgs::Point carPose_pos = carPose.position;
    double carPose_yaw = getYawFromPose(carPose);
    geometry_msgs::Point forwardPt;
    geometry_msgs::Point odom_car2WayPtVec;
    foundForwardPt = false;

    if (!goal_reached)
    {
        for (int i = 0; i < map_path.poses.size(); i++)
        {
            geometry_msgs::PoseStamped map_path_pose = map_path.poses[i];
            geometry_msgs::PoseStamped odom_path_pose;

            try
            {
                tf_listener.transformPose("odom", ros::Time(0), map_path_pose, "map", odom_path_pose);
                geometry_msgs::Point odom_path_wayPt = odom_path_pose.pose.position;
                bool _isForwardWayPt = isForwardWayPt(odom_path_wayPt, carPose);

                if (_isForwardWayPt)
                {
                    bool _isWayPtAwayFromLfwDist = isWayPtAwayFromLfwDist(odom_path_wayPt, carPose_pos);
                    if (_isWayPtAwayFromLfwDist)
                    {
                        forwardPt = odom_path_wayPt;
                        foundForwardPt = true;
                        break;
                    }
                }
            }
            catch (tf::TransformException &ex)
            {
                //ROS_ERROR("%s",ex.what());
                ros::Duration(0.03).sleep();
            }
        }
    }
    else if (goal_reached)
    {
        forwardPt = odom_goal_pos;
        foundForwardPt = false;
        //ROS_INFO("goal REACHED!");
    }

    /*Visualized Target Point on RVIZ*/
    /*Clear former target point Marker*/
    points.points.clear();
    line_strip.points.clear();

    if (foundForwardPt && !goal_reached)
    {
        points.points.push_back(carPose_pos);
        points.points.push_back(forwardPt);
        line_strip.points.push_back(carPose_pos);
        line_strip.points.push_back(forwardPt);
    }

    marker_pub.publish(points);
    marker_pub.publish(line_strip);

    odom_car2WayPtVec.x = cos(carPose_yaw) * (forwardPt.x - carPose_pos.x) + sin(carPose_yaw) * (forwardPt.y - carPose_pos.y);
    odom_car2WayPtVec.y = -sin(carPose_yaw) * (forwardPt.x - carPose_pos.x) + cos(carPose_yaw) * (forwardPt.y - carPose_pos.y);
    return odom_car2WayPtVec;
}

double L1Controller::getEta(const geometry_msgs::Pose &carPose)
{
    geometry_msgs::Point odom_car2WayPtVec = get_odom_car2WayPtVec(carPose);

    double eta = atan2(odom_car2WayPtVec.y, odom_car2WayPtVec.x);
    return eta;
}

double L1Controller::getCar2GoalDist()
{
    geometry_msgs::Point car_pose = odom.pose.pose.position;
    double car2goal_x = odom_goal_pos.x - car_pose.x;
    double car2goal_y = odom_goal_pos.y - car_pose.y;

    double dist2goal = sqrt(car2goal_x * car2goal_x + car2goal_y * car2goal_y);

    return dist2goal;
}

double L1Controller::getL1Distance(const double &_Vcmd)
{
    double L1 = 0;
    if (_Vcmd < 1.34)
        L1 = 3 / 3.2;
    else if (_Vcmd > 1.34 && _Vcmd < 5.36)
        L1 = _Vcmd * 2.24 / 3.0;
    else
        L1 = 12 / 3.0;
    return L1;
}

double L1Controller::getSteeringAngle(double eta)
{
    double steeringAnge = -atan2((L * sin(eta)), (Lfw / 2 + lfw * cos(eta))) * (180.0 / PI);
    //ROS_INFO("Steering Angle = %.2f", steeringAnge);
    return steeringAnge;
}

double L1Controller::getGasInput(const float &current_v)
{
    double u = (Vcmd - current_v) * Gas_gain;
    //ROS_INFO("velocity = %.2f\tu = %.2f",current_v, u);
    return u;
}

void L1Controller::goalReachingCB(const ros::TimerEvent &)
{

    if (goal_received)
    {
        double car2goal_dist = getCar2GoalDist();
        if (car2goal_dist < goalRadius)
        {
            ReStart();
            ROS_INFO("Goal Reached !");
        }
    }
}

void L1Controller::controlLoopCB(const ros::TimerEvent &)
{

    geometry_msgs::Pose carPose = odom.pose.pose;
    geometry_msgs::Twist carVel = odom.twist.twist;
    cmd_vel.linear.x = 5000;
    cmd_vel.angular.z = baseAngle;

    if (goal_received && go_)
    {
        /*Estimate Steering Angle*/
        double eta = getEta(carPose);
        if (foundForwardPt)
        {
            double get_eta = getSteeringAngle(eta) * Angle_gain;
            ROS_INFO("\n get_eta is %.2f\n*********\n", get_eta);
            cmd_vel.angular.z = baseAngle + get_eta;
            /*Estimate Gas Input*/
            if (!goal_reached)
            {
                if (loop_ < start_loop_)
                {
                    loop_++;
                    if (start_speed_ < baseSpeed)
                    {
                        start_speed_ = start_speed_ + 2;
                        now_speed_ = (int)start_speed_;
                        cmd_vel.linear.x = now_speed_;
                    }
                    else
                    {
                        // double u = getGasInput(carVel.linear.x);
                        // cmd_vel.linear.x = baseSpeed - u;
                        loop_ = start_loop_;
                        cmd_vel.linear.x = now_speed_;
                    }
                }
                else
                {
                    // double u = getGasInput(carVel.linear.x);
                    // cmd_vel.linear.x = baseSpeed - u;
                    
                    //abs get_eta;
                    if (get_eta < 0.0)
                    {
                        get_eta = -get_eta;
                    }

                    if (get_eta >= 20.0 && get_eta <= 40.0)
                    { 
                        if(now_speed_ <= 5170) 
                        {
                            now_speed_ = 5170;
                        }  
                        else
                        {
                            now_speed_ = now_speed_ - 5;
                        }                    
                        Lfw = goalRadius = getL1Distance(1.35);
                    }
                    else if (get_eta > 40.0)
                    {
                        if(now_speed_ <= 5150)
                        {
                            now_speed_ = 5150;
                        }   
                        else
                        {
                            now_speed_ = now_speed_ - 5;
                        }            
                        Lfw = goalRadius = getL1Distance(1.2);
                    }
                    else
                    {
                        if(now_speed_ >= baseSpeed)
                        {
                            now_speed_ = baseSpeed;
                        }  
                        else
                        {
                            now_speed_ = now_speed_ + 5;
                        }          
                        Lfw = goalRadius = getL1Distance(Vcmd);
                    }
                    
                    cmd_vel.linear.x = now_speed_; 
                }
                ROS_INFO("\nGas = %.2f\nSteering angle = %.2f", cmd_vel.linear.x, cmd_vel.angular.z);
            }
        }
    }
    pub_.publish(cmd_vel);
}

void Command();
char command = '0';

/*****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "L1Controller_v2");

    auto command_thread = std::thread(Command);
    L1Controller controller;

    while (ros::ok())
    {
        switch (command)
        {
        case '1':
            controller.GoCar();
            command = '0';
            break;
        case '2':
            controller.ReStart();
            command = '0';
            break;
        case 27:
            if (command_thread.joinable())
            {
                command_thread.join();
            }
            return 0;
        default:
            break;
        }
        ros::spinOnce();
    }

    return 0;
}

void Command()
{
    while (command != 27)
    {
        std::cout << "**************************************" << std::endl;
        std::cout << "*********please send a command********" << std::endl;
        std::cout << "> "<< std::endl;
        std::cout << "1: GO"<< std::endl;
        std::cout << "2: ReStart"<< std::endl;
        std::cout << "esc: exit program" << std::endl;
        std::cout << "**************************************" << std::endl;

        std::cin >> command;
    }
}
