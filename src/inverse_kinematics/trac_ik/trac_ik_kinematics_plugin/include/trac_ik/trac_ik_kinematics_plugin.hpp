#pragma once

#include <kdl/chain.hpp>

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/pose.hpp>
#include <string>
#include <vector>

#include <trac_ik_parameters.hpp>

namespace trac_ik_kinematics_plugin
{

class TracIKKinematicsPlugin : public kinematics::KinematicsBase
{
  std::vector<std::string> joint_names_;
  std::vector<std::string> link_names_;

  uint num_joints_;
  bool active_; // Internal variable that indicates whether solvers are configured and ready

  KDL::Chain chain;
  KDL::JntArray joint_min, joint_max;

public:
  const std::vector<std::string>& getJointNames() const
  {
    return joint_names_;
  }
  const std::vector<std::string>& getLinkNames() const
  {
    return link_names_;
  }


  /** @class
   *  @brief Interface for an TRAC-IK kinematics plugin
   */
  TracIKKinematicsPlugin(): active_(false) {}

  ~TracIKKinematicsPlugin()
  {
  }

  /**
   * @brief Given a desired pose of the end-effector, compute the joint angles to reach it
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param solution the solution vector
   * @param error_code an error code that encodes the reason for failure or success
   * @return True if a valid solution was found, false otherwise
   */

  // Returns the first IK solution that is within joint limits, this is called by get_ik() service
  bool getPositionIK(const geometry_msgs::msg::Pose &ik_pose,
                     const std::vector<double> &ik_seed_state,
                     std::vector<double> &solution,
                     moveit_msgs::msg::MoveItErrorCodes &error_code,
                     const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @return True if a valid solution was found, false otherwise
   */
  bool searchPositionIK(const geometry_msgs::msg::Pose &ik_pose,
                        const std::vector<double> &ik_seed_state,
                        double timeout,
                        std::vector<double> &solution,
                        moveit_msgs::msg::MoveItErrorCodes &error_code,
                        const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param the distance that the redundancy can be from the current position
   * @return True if a valid solution was found, false otherwise
   */
  bool searchPositionIK(const geometry_msgs::msg::Pose &ik_pose,
                        const std::vector<double> &ik_seed_state,
                        double timeout,
                        const std::vector<double> &consistency_limits,
                        std::vector<double> &solution,
                        moveit_msgs::msg::MoveItErrorCodes &error_code,
                        const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @return True if a valid solution was found, false otherwise
   */
  bool searchPositionIK(const geometry_msgs::msg::Pose &ik_pose,
                        const std::vector<double> &ik_seed_state,
                        double timeout,
                        std::vector<double> &solution,
                        const IKCallbackFn &solution_callback,
                        moveit_msgs::msg::MoveItErrorCodes &error_code,
                        const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).  The consistency_limit specifies that only certain redundancy positions
   * around those specified in the seed state are admissible and need to be searched.
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param consistency_limit the distance that the redundancy can be from the current position
   * @return True if a valid solution was found, false otherwise
   */
  bool searchPositionIK(const geometry_msgs::msg::Pose &ik_pose,
                        const std::vector<double> &ik_seed_state,
                        double timeout,
                        const std::vector<double> &consistency_limits,
                        std::vector<double> &solution,
                        const IKCallbackFn &solution_callback,
                        moveit_msgs::msg::MoveItErrorCodes &error_code,
                        const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

  bool searchPositionIK(const geometry_msgs::msg::Pose &ik_pose,
                        const std::vector<double> &ik_seed_state,
                        double timeout,
                        std::vector<double> &solution,
                        const IKCallbackFn &solution_callback,
                        moveit_msgs::msg::MoveItErrorCodes &error_code,
                        const std::vector<double> &consistency_limits,
                        const kinematics::KinematicsQueryOptions &options) const;


  /**
   * @brief Given a set of joint angles and a set of links, compute their pose
   *
   * This FK routine is only used if 'use_plugin_fk' is set in the 'arm_kinematics_constraint_aware' node,
   * otherwise ROS TF is used to calculate the forward kinematics
   *
   * @param link_names A set of links for which FK needs to be computed
   * @param joint_angles The state for which FK is being computed
   * @param poses The resultant set of poses (in the frame returned by getBaseFrame())
   * @return True if a valid solution was found, false otherwise
   */
  bool getPositionFK(const std::vector<std::string> &link_names,
                     const std::vector<double> &joint_angles,
                     std::vector<geometry_msgs::msg::Pose> &poses) const;
  
  bool initialize(const rclcpp::Node::SharedPtr& node, 
                  const moveit::core::RobotModel& robot_model,
                  const std::string& group_name, 
                  const std::string& base_frame,
                  const std::vector<std::string>& tip_frames, 
                  double search_discretization) override;

protected:

  trac_ik::Params params_;

  int getKDLSegmentIndex(const std::string &name) const;

}; // end class
}
