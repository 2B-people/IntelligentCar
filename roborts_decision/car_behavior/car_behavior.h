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
  CarBehavior(ChassisExecutor* &chassis_executor,
              Blackboard* &blackboard,
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

    cancel_goal_ = false;
  }
  void Run()
  {
    auto executor_state = Update();
    auto robot_map_pose = blackboard_->GetRobotMapPose();
    if (executor_state != BehaviorState::RUNNING)
    {
      if (!cancel_goal_)
      {
        cancel_goal_ = true;
        chase_goal_.pose.orientation.x = 0;
        chase_goal_.pose.orientation.y = 0;
        chase_goal_.pose.orientation.z = 1;
        chase_goal_.pose.orientation.w = 0.044;

        chase_goal_.pose.position.x = -1.33384430408;
        chase_goal_.pose.position.y = -1.809684515;
        chase_goal_.pose.position.z = 0;
        chassis_executor_->Execute(chase_goal_);
      }
      else
      {
        return;
      }
    }
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

  //! cancel flag
  bool cancel_goal_;
};

} // namespace robort_decision

#endif