#ifndef CAR_BEHAVIOR_H
#define CAR_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

// #include "example_behavior/line_iterator.h"

namespace roborts_decision
{
class CarBehavior
{
public:
  CarBehavior(ChassisExecutor *&chassis_executor,
              Blackboard *&blackboard,
              const std::string &proto_file_path) : chassis_executor_(chassis_executor),
                                                    blackboard_(blackboard)
  {
    chase_goal_.header.frame_id = "map";
    chase_goal_.pose.orientation.x = 0;
    chase_goal_.pose.orientation.y = 0;
    chase_goal_.pose.orientation.z = 0;
    chase_goal_.pose.orientation.w = 1;

    chase_goal_.pose.position.x = 0;
    chase_goal_.pose.position.y = 0;
    chase_goal_.pose.position.z = 0;

    loop_ = 0;

    ros_rviz_sub_ = ros_nh_.subscribe("/move_base_simple/goal", 1, &CarBehavior::RvizGoalCB, this);

    cancel_goal_ = false;
  }
  void Run()
  {
    auto executor_state = Update();
    // auto robot_map_pose = blackboard_->GetRobotMapPose();
    if (executor_state != BehaviorState::RUNNING)
    {
      if (cancel_goal_)
      {
          chassis_executor_->Cancel();
          chase_goal_.header.stamp = ros::Time::now();
          chassis_executor_->Execute(chase_goal_);
      }
    }
  }

  void RvizGoalCB(const geometry_msgs::PoseStamped::ConstPtr &vel)
  {
    chassis_executor_->Cancel();
    chase_goal_.pose.orientation.x = vel->pose.orientation.x;
    chase_goal_.pose.orientation.y = vel->pose.orientation.y;
    chase_goal_.pose.orientation.z = vel->pose.orientation.z;
    chase_goal_.pose.orientation.w = vel->pose.orientation.w;

    chase_goal_.pose.position.x = vel->pose.position.x;
    chase_goal_.pose.position.y = vel->pose.position.y;
    chase_goal_.pose.position.z = vel->pose.position.z;
    cancel_goal_ = true;
  }

  void Cancel()
  {
    chassis_executor_->Cancel();
  }

  BehaviorState Update()
  {
    return chassis_executor_->Update();
  }

  void SetGoal(geometry_msgs::PoseStamped chase_goal)
  {
    chase_goal_ = chase_goal;
  }

  ~CarBehavior() = default;

private:
  //! executor
  ChassisExecutor *const chassis_executor_;

  //! perception information
  Blackboard *const blackboard_;

  //! chase goal
  geometry_msgs::PoseStamped chase_goal_;
  //! ros node handler
  ros::NodeHandle ros_nh_;
  //! ros subscriber for speed control
  ros::Subscriber ros_rviz_sub_;

  //! cancel flag
  bool cancel_goal_;
  int loop_;
};

} // namespace roborts_decision

#endif