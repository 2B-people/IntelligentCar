#include <ros/ros.h>

#include "executor/chassis_executor.h"

#include "example_behavior/back_boot_area_behavior.h"
#include "example_behavior/escape_behavior.h"
#include "example_behavior/chase_behavior.h"
#include "example_behavior/search_behavior.h"
#include "example_behavior/patrol_behavior.h"
#include "example_behavior/goal_behavior.h"
#include "car_behavior/car_behavior.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "behavior_test_node");
  std::string full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";

  auto chassis_executor = new roborts_decision::ChassisExecutor;
  auto blackboard = new roborts_decision::Blackboard(full_path);

  roborts_decision::CarBehavior CarBehavior(chassis_executor, blackboard, full_path);

  ros::Rate rate(10);
  while (ros::ok())
  {
    CarBehavior.Run();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
