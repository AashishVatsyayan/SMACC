/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <move_group_interface_client/client_behaviors/cb_move_end_effector.h>
// #include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/kinematic_constraints/utils.h>
#include <future>

namespace cl_move_group_interface
{
CbMoveEndEffector::CbMoveEndEffector()
{
}

CbMoveEndEffector::CbMoveEndEffector(geometry_msgs::PoseStamped target_pose, std::string tip_link, moveit_msgs::Constraints constraints)
  : targetPose(target_pose), tip_link_(tip_link), constraints(constraints)
{
  // tip_link_ = tip_link;
}

void CbMoveEndEffector::onEntry()
{
  this->requiresClient(movegroupClient_);

  if (this->group_)
  {
      ROS_DEBUG("[CbMoveEndEfector] new thread started to move absolute end effector");
      ROS_ERROR("Starting a new move_group");
      moveit::planning_interface::MoveGroupInterface move_group(*(this->group_));
      this->moveToAbsolutePose(move_group, targetPose, constraints);
      ROS_DEBUG("[CbMoveEndEfector] to move absolute end effector thread destroyed");
  }
  else
  {
      ROS_DEBUG("[CbMoveEndEfector] new thread started to move absolute end effector");
      ROS_ERROR("Resuing a new move_group");
      this->moveToAbsolutePose(movegroupClient_->moveGroupClientInterface, targetPose, constraints);
      ROS_DEBUG("[CbMoveEndEfector] to move absolute end effector thread destroyed");
  }
}

// TODO: Make a pull request to clear this up.
// Added by Aashish on 22/2/2021
bool CbMoveEndEffector::moveToAbsolutePose(moveit::planning_interface::MoveGroupInterface &moveGroupInterface,
                                           geometry_msgs::PoseStamped &targetObjectPose, moveit_msgs::Constraints &constraints)
{
  auto& planningSceneInterface = movegroupClient_->planningSceneInterface;
  ROS_DEBUG("[CbMoveEndEffector] Synchronous sleep of 1 seconds");
  // ros::Duration(0.5).sleep();

  moveGroupInterface.setPlanningTime(1.0);

  ROS_ERROR_STREAM("[CbMoveEndEffector] Target End efector Pose: " << targetObjectPose);
  // ROS_ERROR_STREAM("TIP LINK IS :" << tip_link_);
  moveGroupInterface.setEndEffectorLink("tcp_frame");

  // moveGroupInterface.setPlannerId("STRIDE");
  // moveGroupInterface.setPlannerId("RRTConnect");
  targetObjectPose.header.frame_id="base_link";
  moveGroupInterface.setPoseReferenceFrame(targetObjectPose.header.frame_id);
  // moveGroupInterface.setPathConstraints(constraints);

  
  moveGroupInterface.setPoseTarget(targetObjectPose, tip_link_);
  
  ROS_ERROR_STREAM("I think the end effector is: "<< moveGroupInterface.getEndEffectorLink());
  ROS_ERROR_STREAM("I base ref  is: "<< targetObjectPose.header.frame_id);
  

  moveit::planning_interface::MoveGroupInterface::Plan computedMotionPlan;
  bool success = (moveGroupInterface.plan(computedMotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("CbMoveEndEffector", "Success Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  if (success)
  {
    auto executionResult = moveGroupInterface.execute(computedMotionPlan);

    if (executionResult == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      ROS_INFO("[CbMoveEndEffector] motion execution succedded");
      movegroupClient_->postEventMotionExecutionSucceded();
      this->postSuccessEvent();

    }
    else
    {
      ROS_INFO("[CbMoveEndEffector] motion execution failed");
      movegroupClient_->postEventMotionExecutionFailed();
      this->postFailureEvent();
    }
  }
  else
  {
    ROS_INFO("[CbMoveEndEffector] motion execution failed");
    movegroupClient_->postEventMotionExecutionFailed();
    this->postFailureEvent();
  }

  ROS_DEBUG("[CbMoveEndEffector] Synchronous sleep of 1 seconds");
  // ros::Duration(0.5).sleep();

  return success;
}

}  // namespace cl_move_group_interface