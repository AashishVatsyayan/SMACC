/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <move_group_interface_client/cl_movegroup.h>
#include <smacc/smacc_asynchronous_client_behavior.h>
#include <future>
namespace cl_move_group_interface
{
class CbMoveEndEffector : public smacc::SmaccAsyncClientBehavior
{
public:
  geometry_msgs::PoseStamped targetPose;
  std::string tip_link_;
  boost::optional<std::string> group_;
  moveit_msgs::Constraints constraints;

  CbMoveEndEffector();
  CbMoveEndEffector(geometry_msgs::PoseStamped target_pose, std::string tip_link, moveit_msgs::Constraints constraints);

  virtual void onEntry() override;

protected:
  bool moveToAbsolutePose(moveit::planning_interface::MoveGroupInterface &moveGroupInterface,
                          geometry_msgs::PoseStamped &targetObjectPose, moveit_msgs::Constraints &constraints);

  ClMoveGroup *movegroupClient_;
};
}  // namespace cl_move_group_interface