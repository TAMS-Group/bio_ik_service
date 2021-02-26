#include <bio_ik/bio_ik.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>

#include <tf_conversions/tf_eigen.h>
#include <tf_conversions/tf_kdl.h>

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <eigen_conversions/eigen_msg.h>

#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/RobotState.h>

#include <bio_ik_msgs/GetIK.h>
#include <bio_ik_msgs/IKRequest.h>
#include <bio_ik_msgs/IKResponse.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <mutex>
#include <thread>
#include <unordered_map>

static moveit::core::RobotModelPtr
getRobotModel(std::string robot_description) {
  if (robot_description.empty())
    robot_description = "robot_description";
  static std::mutex mutex;
  std::lock_guard<std::mutex> lock(mutex);
  static std::unordered_map<std::string, moveit::core::RobotModelPtr>
      robot_models;
  if (robot_models.find(robot_description) == robot_models.end()) {
    ROS_INFO("loading robot model %s", robot_description.c_str());
    static std::unordered_map<std::string, robot_model_loader::RobotModelLoader>
        loaders;
    loaders.emplace(robot_description, robot_description);
    robot_models[robot_description] = loaders[robot_description].getModel();
    if (!robot_models[robot_description]) {
      ROS_ERROR("failed to load robot model %s", robot_description.c_str());
    }
  }
  return robot_models[robot_description];
}

static planning_scene::PlanningSceneConstPtr
getPlanningScene(std::string robot_description) {
  if (robot_description.empty())
    robot_description = "robot_description";
  static std::mutex mutex;
  std::lock_guard<std::mutex> lock(mutex);
  static std::unordered_map<std::string,
                            planning_scene_monitor::PlanningSceneMonitorPtr>
      planning_scene_monitors;
  if (planning_scene_monitors.find(robot_description) ==
      planning_scene_monitors.end()) {
    ROS_INFO("connecting to planning scene");
    planning_scene_monitors[robot_description] =
        planning_scene_monitor::PlanningSceneMonitorPtr(
            new planning_scene_monitor::PlanningSceneMonitor(
                robot_description));
  }
  planning_scene::PlanningSceneConstPtr planning_scene =
      planning_scene_monitors[robot_description]->getPlanningScene();
  if (!planning_scene) {
    ROS_ERROR_ONCE("failed to connect to planning scene");
  }
  return planning_scene;
}

static bool getPositionIK(moveit_msgs::GetPositionIK::Request &request,
                          moveit_msgs::GetPositionIK::Response &response,
                          bool approximate) {
  auto robot_model = getRobotModel("");
  if (!robot_model) {
    response.error_code.val =
        moveit_msgs::MoveItErrorCodes::INVALID_OBJECT_NAME;
    return true;
  }

  auto joint_model_group =
      robot_model->getJointModelGroup(request.ik_request.group_name);
  if (!joint_model_group) {
    response.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    return true;
  }

  static std::mutex mutex;
  std::lock_guard<std::mutex> lock(mutex);

  moveit::core::RobotState robot_state(robot_model);

  robot_state.setToDefaultValues();
  robot_state.update();

  if (!request.ik_request.robot_state.joint_state.name.empty() ||
      !request.ik_request.robot_state.multi_dof_joint_state.joint_names
           .empty()) {
    moveit::core::robotStateMsgToRobotState(request.ik_request.robot_state,
                                            robot_state);
    robot_state.update();
  }

  bool success = true;

  bio_ik::BioIKKinematicsQueryOptions ik_options;
  ik_options.return_approximate_solution = approximate;

  moveit::core::GroupStateValidityCallbackFn callback;
  if (request.ik_request.avoid_collisions) {
    callback = [](moveit::core::RobotState *state,
                  const moveit::core::JointModelGroup *group,
                  const double *values) {
      auto planning_scene = getPlanningScene("");
      state->setJointGroupPositions(group, values);
      state->update();
      return !planning_scene ||
             !planning_scene->isStateColliding(*state, group->getName());
    };
  }

  if (request.ik_request.pose_stamped_vector.empty()) {
    if (request.ik_request.ik_link_name.empty()) {
      success = robot_state.setFromIK(
          joint_model_group, request.ik_request.pose_stamped.pose,
          request.ik_request.timeout.toSec(),
          callback, ik_options);
    } else {
      success = robot_state.setFromIK(
          joint_model_group, request.ik_request.pose_stamped.pose,
          request.ik_request.ik_link_name, request.ik_request.timeout.toSec(),
          callback, ik_options);
    }
  } else {
    EigenSTL::vector_Isometry3d poses;
    poses.reserve(request.ik_request.pose_stamped_vector.size());
    for (auto &pose : request.ik_request.pose_stamped_vector) {
      poses.emplace_back();
      tf::poseMsgToEigen(pose.pose, poses.back());
    }
    if (request.ik_request.ik_link_names.empty()) {
      std::vector<std::string> end_effector_names;
      joint_model_group->getEndEffectorTips(end_effector_names);
      success = robot_state.setFromIK(
          joint_model_group, poses, end_effector_names,
          request.ik_request.timeout.toSec(),
          callback, ik_options);
    } else {
      success = robot_state.setFromIK(
          joint_model_group, poses, request.ik_request.ik_link_names,
          request.ik_request.timeout.toSec(),
          callback, ik_options);
    }
  }

  robot_state.update();

  moveit::core::robotStateToRobotStateMsg(robot_state, response.solution);
  if (success) {
    response.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  } else {
    response.error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
  }
  return true;
}

static bool getPositionIK(moveit_msgs::GetPositionIK::Request &request,
                          moveit_msgs::GetPositionIK::Response &response) {
  return getPositionIK(request, response, false);
}

static bool getApproximateIK(moveit_msgs::GetPositionIK::Request &request,
                             moveit_msgs::GetPositionIK::Response &response) {
  return getPositionIK(request, response, true);
}

static tf2::Vector3 p(const geometry_msgs::Point &p) {
  return tf2::Vector3(p.x, p.y, p.z);
}

static tf2::Vector3 p(const geometry_msgs::Vector3 &p) {
  return tf2::Vector3(p.x, p.y, p.z);
}

static tf2::Quaternion q(const geometry_msgs::Quaternion &q) {
  return tf2::Quaternion(q.x, q.y, q.z, q.w);
}

static double w(double w, double def = 1.0) {
  if (w == 0 || !std::isfinite(w))
    w = def;
  return w;
}

static void convertGoals(const bio_ik_msgs::IKRequest &ik_request,
                         bio_ik::BioIKKinematicsQueryOptions &ik_options) {
  for (auto &m : ik_request.position_goals) {
    ik_options.goals.emplace_back(
        new bio_ik::PositionGoal(m.link_name, p(m.position), w(m.weight)));
  }

  for (auto &m : ik_request.orientation_goals) {
    ik_options.goals.emplace_back(new bio_ik::OrientationGoal(
        m.link_name, q(m.orientation), w(m.weight)));
  }

  for (auto &m : ik_request.pose_goals) {
    auto *g = new bio_ik::PoseGoal(m.link_name, p(m.pose.position),
                                   q(m.pose.orientation), w(m.weight));
    g->setRotationScale(w(m.rotation_scale, 0.5));
    ik_options.goals.emplace_back(g);
  }

  for (auto &m : ik_request.look_at_goals) {
    ik_options.goals.emplace_back(new bio_ik::LookAtGoal(
        m.link_name, p(m.axis), p(m.target), w(m.weight)));
  }

  for (auto &m : ik_request.min_distance_goals) {
    ik_options.goals.emplace_back(new bio_ik::MinDistanceGoal(
        m.link_name, p(m.target), m.distance, w(m.weight)));
  }

  for (auto &m : ik_request.max_distance_goals) {
    ik_options.goals.emplace_back(new bio_ik::MaxDistanceGoal(
        m.link_name, p(m.target), m.distance, w(m.weight)));
  }

  for (auto &m : ik_request.line_goals) {
    ik_options.goals.emplace_back(new bio_ik::LineGoal(
        m.link_name, p(m.position), p(m.direction), w(m.weight)));
  }

#if (MOVEIT_FCL_VERSION < FCL_VERSION_CHECK(0, 6, 0))
  for (auto &m : ik_request.touch_goals) {
    ik_options.goals.emplace_back(new bio_ik::TouchGoal(
        m.link_name, p(m.position), p(m.normal), w(m.weight)));
  }
#else
  if (ik_request.touch_goals.size() > 0) {
    ROS_WARN("TouchGoals are not supported with the current FCL version");
  }
#endif

  for (auto &m : ik_request.avoid_joint_limits_goals) {
    ik_options.goals.emplace_back(
        new bio_ik::AvoidJointLimitsGoal(w(m.weight), !m.primary));
  }

  for (auto &m : ik_request.minimal_displacement_goals) {
    ik_options.goals.emplace_back(
        new bio_ik::MinimalDisplacementGoal(w(m.weight), !m.primary));
  }

  for (auto &m : ik_request.center_joints_goals) {
    ik_options.goals.emplace_back(
        new bio_ik::CenterJointsGoal(w(m.weight), !m.primary));
  }

  for (auto &m : ik_request.joint_variable_goals) {
    ik_options.goals.emplace_back(new bio_ik::JointVariableGoal(
        m.variable_name, m.variable_position, w(m.weight), m.secondary));
  }

  for (auto &m : ik_request.balance_goals) {
    auto *g = new bio_ik::BalanceGoal(p(m.target), w(m.weight));
    if (m.axis.x || m.axis.y || m.axis.z) {
      g->setAxis(p(m.axis));
    }
    ik_options.goals.emplace_back(g);
  }

  for (auto &m : ik_request.side_goals) {
    ik_options.goals.emplace_back(new bio_ik::SideGoal(
        m.link_name, p(m.axis), p(m.direction), w(m.weight)));
  }

  for (auto &m : ik_request.direction_goals) {
    ik_options.goals.emplace_back(new bio_ik::DirectionGoal(
        m.link_name, p(m.axis), p(m.direction), w(m.weight)));
  }

  for (auto &m : ik_request.cone_goals) {
    ik_options.goals.emplace_back(
        new bio_ik::ConeGoal(m.link_name, p(m.position), w(m.position_weight),
                             p(m.axis), p(m.direction), m.angle, w(m.weight)));
  }
}

static bool getBioIK(bio_ik_msgs::GetIK::Request &request,
                     bio_ik_msgs::GetIK::Response &response) {
  auto robot_model = getRobotModel("");
  if (!robot_model) {
    response.ik_response.error_code.val =
        moveit_msgs::MoveItErrorCodes::INVALID_OBJECT_NAME;
    return true;
  }

  auto joint_model_group =
      robot_model->getJointModelGroup(request.ik_request.group_name);
  if (!joint_model_group) {
    response.ik_response.error_code.val =
        moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    return true;
  }

  static std::mutex mutex;
  std::lock_guard<std::mutex> lock(mutex);

  moveit::core::RobotState robot_state(robot_model);

  robot_state.setToDefaultValues();
  robot_state.update();

  if (!request.ik_request.robot_state.joint_state.name.empty() ||
      !request.ik_request.robot_state.multi_dof_joint_state.joint_names
           .empty()) {
    moveit::core::robotStateMsgToRobotState(request.ik_request.robot_state,
                                            robot_state);
    robot_state.update();
  }

  bio_ik::BioIKKinematicsQueryOptions ik_options;
  ik_options.return_approximate_solution = request.ik_request.approximate;
  ik_options.fixed_joints=request.ik_request.fixed_joints;
  ik_options.replace = true;

  convertGoals(request.ik_request, ik_options);

  moveit::core::GroupStateValidityCallbackFn callback;
  if (request.ik_request.avoid_collisions) {
    callback = [](moveit::core::RobotState *state,
                  const moveit::core::JointModelGroup *group,
                  const double *values) {
      auto planning_scene = getPlanningScene("");
      state->setJointGroupPositions(group, values);
      state->update();
      return !planning_scene ||
             !planning_scene->isStateColliding(*state, group->getName());
    };
  }

  bool success = robot_state.setFromIK(
      joint_model_group, EigenSTL::vector_Isometry3d(),
      std::vector<std::string>(), request.ik_request.timeout.toSec(),
      callback, ik_options);

  robot_state.update();

  moveit::core::robotStateToRobotStateMsg(robot_state,
                                          response.ik_response.solution);
  response.ik_response.solution_fitness = ik_options.solution_fitness;
  if (success) {
    response.ik_response.error_code.val =
        moveit_msgs::MoveItErrorCodes::SUCCESS;
  } else {
    response.ik_response.error_code.val =
        moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
  }
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "bio_ik");

  ROS_INFO("bio ik service");

  ros::NodeHandle node_handle;

  ros::ServiceServer get_position_ik =
      node_handle.advertiseService("bio_ik/get_position_ik", &getPositionIK);

  ros::ServiceServer get_approximate_ik = node_handle.advertiseService(
      "bio_ik/get_approximate_ik", &getApproximateIK);

  ros::ServiceServer get_bio_ik =
      node_handle.advertiseService("bio_ik/get_bio_ik", &getBioIK);

  getRobotModel("");

  getPlanningScene("");

  ros::MultiThreadedSpinner spinner;
  spinner.spin();

  return 0;
}
