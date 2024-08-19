#include <algorithm>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <trac_ik/trac_ik.hpp>
#include <limits>
#include <fmt/format.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_kdl/tf2_kdl.hpp>

#include <trac_ik/trac_ik_kinematics_plugin.hpp>

#include <moveit/utils/moveit_error_code.h>

namespace trac_ik_kinematics_plugin
{

namespace 
{

auto const LOGGER = rclcpp::get_logger("trac_ik");

// rclcpp::Logger getLogger() { return moveit::getLogger("trac_ik"); }

} // namespace

bool TracIKKinematicsPlugin::initialize(
                const rclcpp::Node::SharedPtr& node, 
                const moveit::core::RobotModel& robot_model,
                const std::string& group_name, 
                const std::string& base_frame,
                const std::vector<std::string>& tip_frames, 
                double search_discretization)
{
  node_ = node;

  // Get Solver Parameters
  std::string kinematics_param_prefix = "robot_description_kinematics." + group_name;
  auto param_listener_ = std::make_shared<trac_ik::ParamListener>(node, kinematics_param_prefix);
  params_ = param_listener_->get_params();

  // Initialize internal state of base class KinematicsBase
  // Creates these internal state variables:
  // robot_model_ <- shared_ptr to RobotModel
  // robot_description_ <- empty string
  // group_name_ <- group_name string
  // base_frame_ <- base_frame without leading /
  // tip_frames_ <- tip_frames without leading /
  // redundant_joint_discretization_ <- vector initialized with
  // search_discretization
  storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);
  
  auto joint_model_group_ = robot_model_->getJointModelGroup(group_name);
  if (!joint_model_group_)
  {
    RCLCPP_ERROR(LOGGER, fmt::format("Failed to get Group \"{}\"", group_name.c_str()).c_str());
    return false;
  }

  if (!joint_model_group_->isChain())
  {
    RCLCPP_ERROR(LOGGER, fmt::format("Group \"{}\" is not a chain", group_name.c_str()).c_str());
    return false;
  }
  if (!joint_model_group_->isSingleDOFJoints())
  {
    RCLCPP_ERROR(LOGGER, fmt::format("Group \"{}\" includes joints that have more than 1 DOF", group_name.c_str()).c_str());
    return false;
  }
  
  RCLCPP_DEBUG_STREAM(LOGGER, "Reading joints and links from URDF");

  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(*robot_model.getURDF(), kdl_tree))
  {
    RCLCPP_FATAL(LOGGER, "Failed to extract kdl tree from urdf model");
    return false;
  }

  const std::string tip_name = getTipFrame();
  if (!kdl_tree.getChain(base_frame, tip_name, chain))
  {
    RCLCPP_FATAL(LOGGER, fmt::format("Couldn't find chain {} to {}", base_frame.c_str(), tip_name.c_str()).c_str());
    return false;
  }

  num_joints_ = chain.getNrOfJoints();

  std::vector<KDL::Segment> chain_segs = chain.segments;

  std::vector<double> l_bounds, u_bounds;

  joint_min.resize(num_joints_);
  joint_max.resize(num_joints_);

  auto urdf_model = robot_model.getURDF();

  uint joint_num = 0;
  for (unsigned int i = 0; i < chain_segs.size(); ++i)
  {
    link_names_.push_back(chain_segs[i].getName());
    // urdf::JointConstSharedPtr joint = robot_model.getJoint(chain_segs[i].getJoint().getName());
    auto joint = urdf_model->getJoint(chain_segs[i].getJoint().getName());
    if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
    {
      joint_num++;
      assert(joint_num <= num_joints_);
      double lower, upper;
      int hasLimits;
      joint_names_.push_back(joint->name);
      if (joint->type != urdf::Joint::CONTINUOUS)
      {
        if (joint->safety)
        {
          lower = std::max(joint->limits->lower, joint->safety->soft_lower_limit);
          upper = std::min(joint->limits->upper, joint->safety->soft_upper_limit);
        }
        else
        {
          lower = joint->limits->lower;
          upper = joint->limits->upper;
        }
        hasLimits = 1;
      }
      else
      {
        hasLimits = 0;
      }
      if (hasLimits)
      {
        joint_min(joint_num - 1) = lower;
        joint_max(joint_num - 1) = upper;
      }
      else
      {
        joint_min(joint_num - 1) = std::numeric_limits<float>::lowest();
        joint_max(joint_num - 1) = std::numeric_limits<float>::max();
      }
      RCLCPP_INFO_STREAM(LOGGER, fmt::format("IK Using joint {}, limits [{}, {}]", 
        chain_segs[i].getName().c_str(), joint_min(joint_num - 1), joint_max(joint_num - 1)));
    }
  }

  params_ = param_listener_->get_params();

  active_ = true;
  return true;
}


int TracIKKinematicsPlugin::getKDLSegmentIndex(const std::string &name) const
{
  unsigned i = 0;
  while (i < chain.getNrOfSegments())
  {
    if (chain.getSegment(i).getName() == name)
    {
      return static_cast<int>(i + 1);
    }
    i++;
  }
  return -1;
}


bool TracIKKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
    const std::vector<double> &joint_angles,
    std::vector<geometry_msgs::msg::Pose> &poses) const
{
  if (!active_)
  {
    RCLCPP_ERROR(LOGGER, "kinematics not active");
    return false;
  }
  poses.resize(link_names.size());
  if (joint_angles.size() != num_joints_)
  {
    RCLCPP_ERROR(LOGGER, fmt::format("Joint angles vector must have size: {}", num_joints_).c_str());
    return false;
  }

  KDL::Frame p_out;
  geometry_msgs::msg::PoseStamped pose;
  // tf::Stamped<tf::Pose> tf_pose;

  KDL::JntArray jnt_pos_in(num_joints_);
  for (unsigned int i = 0; i < num_joints_; i++)
  {
    jnt_pos_in(i) = joint_angles[i];
  }

  KDL::ChainFkSolverPos_recursive fk_solver(chain);

  bool valid = true;
  for (unsigned int i = 0; i < poses.size(); i++)
  {
    RCLCPP_DEBUG(LOGGER, fmt::format("End effector index: {}", getKDLSegmentIndex(link_names[i])).c_str());
    if (fk_solver.JntToCart(jnt_pos_in, p_out, getKDLSegmentIndex(link_names[i])) >= 0)
    {
      tf2::fromMsg(p_out, poses[i]);
    }
    else
    {
      RCLCPP_ERROR(LOGGER, fmt::format("Could not compute FK for {}", link_names[i].c_str()).c_str());
      valid = false;
    }
  }

  return valid;
}


bool TracIKKinematicsPlugin::getPositionIK(const geometry_msgs::msg::Pose &ik_pose,
    const std::vector<double> &ik_seed_state,
    std::vector<double> &solution,
    moveit_msgs::msg::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const
{
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          default_timeout_,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool TracIKKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose &ik_pose,
    const std::vector<double> &ik_seed_state,
    double timeout,
    std::vector<double> &solution,
    moveit_msgs::msg::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const
{
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool TracIKKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose &ik_pose,
    const std::vector<double> &ik_seed_state,
    double timeout,
    const std::vector<double> &consistency_limits,
    std::vector<double> &solution,
    moveit_msgs::msg::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const
{
  const IKCallbackFn solution_callback = 0;
  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool TracIKKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose &ik_pose,
    const std::vector<double> &ik_seed_state,
    double timeout,
    std::vector<double> &solution,
    const IKCallbackFn &solution_callback,
    moveit_msgs::msg::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const
{
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool TracIKKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose &ik_pose,
    const std::vector<double> &ik_seed_state,
    double timeout,
    const std::vector<double> &consistency_limits,
    std::vector<double> &solution,
    const IKCallbackFn &solution_callback,
    moveit_msgs::msg::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const
{
  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool TracIKKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose &ik_pose,
    const std::vector<double> &ik_seed_state,
    double timeout,
    std::vector<double> &solution,
    const IKCallbackFn &solution_callback,
    moveit_msgs::msg::MoveItErrorCodes &error_code,
    const std::vector<double> &/*consistency_limits*/,
    const kinematics::KinematicsQueryOptions &/*options*/) const
{
  RCLCPP_DEBUG_STREAM(LOGGER, "getPositionIK");

  if (!active_)
  {
    RCLCPP_ERROR(LOGGER, "kinematics not active");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  if (ik_seed_state.size() != num_joints_)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Seed state must have size " << num_joints_ << " instead of size " << ik_seed_state.size());
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  KDL::Frame frame;
  tf2::fromMsg(ik_pose, frame);

  KDL::JntArray in(num_joints_), out(num_joints_);

  for (uint z = 0; z < num_joints_; z++)
    in(z) = ik_seed_state[z];

  KDL::Twist bounds = KDL::Twist::Zero();

  if (params_.position_only_ik)
  {
    bounds.rot.x(std::numeric_limits<float>::max());
    bounds.rot.y(std::numeric_limits<float>::max());
    bounds.rot.z(std::numeric_limits<float>::max());
  }

  double epsilon = 1e-5;  //Same as MoveIt's KDL plugin

  TRAC_IK::SolveType solve_type;

  if (params_.solve_type.compare("Manipulation1") == 0)
    solve_type = TRAC_IK::Manip1;
  else if (params_.solve_type.compare("Manipulation2") == 0)
    solve_type = TRAC_IK::Manip2;
  else if (params_.solve_type.compare("Distance") == 0)
    solve_type = TRAC_IK::Distance;
  else
  {
    if (params_.solve_type.compare("Speed") != 0)
    {
      RCLCPP_WARN_STREAM(LOGGER, params_.solve_type << " is not a valid solve_type; setting to default: Speed");
    }
    solve_type = TRAC_IK::Speed;
  }

  TRAC_IK::TRAC_IK ik_solver(chain, joint_min, joint_max, timeout, epsilon, solve_type);

  int rc = ik_solver.CartToJnt(in, frame, out, bounds);

  solution.resize(num_joints_);

  if (rc >= 0)
  {
    for (uint z = 0; z < num_joints_; z++)
      solution[z] = out(z);

    // check for collisions if a callback is provided
    if (solution_callback)
    {
      solution_callback(ik_pose, solution, error_code);
      if (error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
      {
        RCLCPP_DEBUG_STREAM(LOGGER, "Solution passes callback");
        return true;
      }
      else
      {
        RCLCPP_DEBUG_STREAM(LOGGER, fmt::format("Solution has error code {} ({})", 
          error_code.val, moveit::core::errorCodeToString(error_code).c_str()));
        return false;
      }
    }
    else
      return true; // no collision check callback provided
  }
  error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
  return false;
}



} // end namespace

//register TracIKKinematicsPlugin as a KinematicsBase implementation

// #include <class_loader/class_loader.hpp>
// CLASS_LOADER_REGISTER_CLASS(trac_ik_kinematics_plugin::TracIKKinematicsPlugin, kinematics::KinematicsBase);

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(trac_ik_kinematics_plugin::TracIKKinematicsPlugin, kinematics::KinematicsBase);

#include <moveit_kinematics/moveit/cached_ik_kinematics_plugin/cached_ik_kinematics_plugin.h>
PLUGINLIB_EXPORT_CLASS(cached_ik_kinematics_plugin::CachedIKKinematicsPlugin<trac_ik_kinematics_plugin::TracIKKinematicsPlugin>, kinematics::KinematicsBase);