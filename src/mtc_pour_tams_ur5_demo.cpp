#include <ros/ros.h>

#include <moveit/robot_model/robot_model.h> // TODO shouldn't be necessary...

#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>

#include <moveit/task_constructor/stages/pour_into.h>

#include <moveit/task_constructor/stages/modify_planning_scene.h>

#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_pose.h>

#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>

#include "mtc_pour/demo_utils.hpp"

using namespace moveit::task_constructor;

int main(int argc, char **argv) {
  ros::init(argc, argv, "mtc_pouring");

  ros::AsyncSpinner spinner(2);
  ros::NodeHandle pnh("~");
  spinner.start();

  if(pnh.param<bool>("spawn_objects", true)) {
    geometry_msgs::PoseStamped bottle, glass;
    bottle.header.frame_id = "table_top";
    bottle.pose.position.x = 0.0;
    bottle.pose.position.y = 0.15;
    bottle.pose.position.z = 0.0;
    bottle.pose.orientation.w = 1.0;

    glass.header.frame_id = "table_top";
    glass.pose.position.x = -0.1;
    glass.pose.position.y = -0.12;
    glass.pose.position.z = 0.0;
    glass.pose.orientation.w = 1.0;

    mtc_pour::cleanup();
    std::vector<moveit_msgs::CollisionObject> objs;
    moveit::planning_interface::PlanningSceneInterface psi;
    mtc_pour::setupObjects(objs, bottle, glass);
    psi.applyCollisionObjects(objs);
  }

  Task t("my_task");
  t.loadRobotModel();

  auto sampling_planner = std::make_shared<solvers::PipelinePlanner>();
  sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

  auto joint_interpolation = std::make_shared<solvers::JointInterpolationPlanner>();

  // TODO: ignored because it is always overruled by Connect's timeout property
  // sampling_planner->setTimeout(15.0);
  // pipeline->setPlannerId("");
  // std::chrono::duration<double> connect_timeout(1.0);
  double connect_timeout(1.0);

  // don't spill liquid
  moveit_msgs::Constraints upright_constraint;
  upright_constraint.name = "s_model_tool0:upright";
  upright_constraint.orientation_constraints.resize(1);
  {
    moveit_msgs::OrientationConstraint &c =
        upright_constraint.orientation_constraints[0];
    c.link_name = "s_model_tool0";
    c.header.frame_id = "table_top";
    c.orientation.w = 1.0;
    c.absolute_x_axis_tolerance = 0.65;
    c.absolute_y_axis_tolerance = 0.65;
    c.absolute_z_axis_tolerance = M_PI;
    c.weight = 1.0;
  }
  bool with_path_constraint = pnh.param<bool>("with_path_constraint", true);
  ROS_INFO(with_path_constraint ? "Using upright constraint" : "Ignoring upright constraint");

  auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(.3);
  cartesian_planner->setMaxAccelerationScalingFactor(.3);
  cartesian_planner->setStepSize(.002);

  t.setProperty("group", "arm");
  t.setProperty("eef", "gripper");
  t.setProperty("gripper", "gripper"); // TODO: use this

  Stage *current_state = nullptr;
  {
    auto _current_state =
        std::make_unique<stages::CurrentState>("current state");
    current_state = _current_state.get();
    t.add(std::move(_current_state));
  }

  {
    auto stage =
        std::make_unique<stages::MoveTo>("open gripper", joint_interpolation);
    stage->setGroup("gripper");
    stage->setGoal("open");
    t.add(std::move(stage));
  }

  {
	 // TODO: overload for single planner case (breaks make_unique variadic parameter inference)
	 auto stage = std::make_unique<stages::Connect>(
	     "move to pre-grasp pose",
	     stages::Connect::GroupPlannerVector{{"arm", sampling_planner}});
	 stage->properties().configureInitFrom(
	     Stage::PARENT); // TODO: convenience-wrapper
	 t.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<stages::MoveRelative>("approach object",
                                                        cartesian_planner);
    stage->setMarkerNS("approach");
    stage->properties().set("link", "s_model_tool0");
    stage->properties().configureInitFrom(Stage::PARENT, {"group"});
    stage->setMinMaxDistance(.05, .15);

    geometry_msgs::Vector3Stamped vec;
    vec.header.frame_id = "s_model_tool0";
    vec.vector.x = 1.0;
    stage->setDirection(vec);
    t.add(std::move(stage));
  }

  {
    auto stage =
        std::make_unique<stages::GenerateGraspPose>("grasp workspace pose");
    stage->properties().configureInitFrom(Stage::PARENT);
    stage->setPreGraspPose("open");
    stage->setObject("bottle");
    stage->setAngleDelta(M_PI / 12);

    stage->setMonitoredStage(current_state);

    auto wrapper =
        std::make_unique<stages::ComputeIK>("grasp pose", std::move(stage));
    wrapper->setMaxIKSolutions(8);
    wrapper->setIKFrame(Eigen::Translation3d(0.05, 0, 0), "s_model_tool0");
    wrapper->setTimeout(0.5);
    // TODO adding this will initialize "target_pose" which is internal (or
    // isn't it?)
    // wrapper->properties().configureInitFrom(Stage::PARENT);
    wrapper->properties().configureInitFrom(
        Stage::PARENT, {"eef"}); // TODO: convenience wrapper
    wrapper->properties().configureInitFrom(Stage::INTERFACE, {"target_pose"});
    t.add(std::move(wrapper));
  }

  {
    auto stage = std::make_unique<stages::ModifyPlanningScene>(
        "allow gripper->object collision");
    stage->allowCollisions("bottle",
                           t.getRobotModel()
                               ->getJointModelGroup("gripper")
                               ->getLinkModelNamesWithCollisionGeometry(),
                           true);
    t.add(std::move(stage));
  }

  {
    auto stage =
        std::make_unique<stages::MoveTo>("close gripper", joint_interpolation);
    stage->properties().property("group").configureInitFrom(
        Stage::PARENT, "gripper"); // TODO this is not convenient
    stage->setGoal("closed");
    t.add(std::move(stage));
  }

  Stage *object_grasped = nullptr;
  {
    auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
    stage->attachObject("bottle", "s_model_tool0");
    object_grasped = stage.get();
    t.add(std::move(stage));
  }

  Stage *object_lifted = nullptr;
  {
    auto stage = std::make_unique<stages::MoveRelative>("lift object",
                                                        cartesian_planner);
    stage->properties().configureInitFrom(Stage::PARENT, {"group"});
    stage->setMinMaxDistance(.08, .13);
    stage->setIKFrame("s_model_tool0"); // TODO property for frame

    stage->setMarkerNS("lift");

    geometry_msgs::Vector3Stamped vec;
    vec.header.frame_id = "table_top";
    vec.vector.z = 1.0;
    stage->setDirection(vec);
    object_lifted = stage.get();
    t.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<stages::Connect>(
        "move to pre-pour pose",
        stages::Connect::GroupPlannerVector{{"arm", sampling_planner}});
    stage->setTimeout(connect_timeout);
    if(with_path_constraint)
      stage->setPathConstraints(upright_constraint);
    stage->properties().configureInitFrom(
        Stage::PARENT); // TODO: convenience-wrapper
    t.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<stages::GeneratePose>("pose above glass");
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "glass";
    p.pose.orientation.w = 1;
    p.pose.position.z = .3;
    stage->setPose(p);
    stage->properties().configureInitFrom(Stage::PARENT);

    stage->setMonitoredStage(object_grasped);

    auto wrapper =
        std::make_unique<stages::ComputeIK>("pre-pour pose", std::move(stage));
    wrapper->setMaxIKSolutions(8);
    wrapper->setTimeout(0.5);
    // TODO adding this will initialize "target_pose" which is internal (or
    // isn't it?)
    // wrapper->properties().configureInitFrom(Stage::PARENT);
    wrapper->properties().configureInitFrom(
        Stage::PARENT, {"eef"}); // TODO: convenience wrapper
    wrapper->properties().configureInitFrom(Stage::INTERFACE, {"target_pose"});
    t.add(std::move(wrapper));
  }

  Stage* pouring = nullptr;
  {
    auto stage = std::make_unique<mtc_pour::PourInto>("pouring");
    stage->setBottle("bottle");
    stage->setContainer("glass");
    stage->setPourOffset(Eigen::Vector3d(0, 0.015, 0.035));
    stage->setTiltAngle(2.0);
    stage->setPourDuration(ros::Duration(4.0));
    {
      geometry_msgs::Vector3Stamped pouring_axis;
      pouring_axis.header.frame_id = "s_model_tool0";
      pouring_axis.vector.x = 1.0;
      stage->setPouringAxis(pouring_axis);
    }
    stage->properties().configureInitFrom(Stage::PARENT, {"group"});
    // TODO: This would have unintuitive results:
    // stage->properties().configureInitFrom(Stage::PARENT);
    // because it includes "marker_ns"
    pouring = stage.get();
    t.add(std::move(stage));
  }

  // PLACE

  {
    auto stage = std::make_unique<stages::Connect>(
        "move to pre-place pose",
        stages::Connect::GroupPlannerVector{{"arm", sampling_planner}});
    stage->setTimeout(connect_timeout);
    if(with_path_constraint)
      stage->setPathConstraints(upright_constraint);
    stage->properties().configureInitFrom(
        Stage::PARENT); // TODO: convenience-wrapper
    t.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<stages::MoveRelative>("put down object",
                                                        cartesian_planner);
    stage->setMarkerNS("approach-place");
    stage->properties().set("link", "s_model_tool0");
    stage->properties().configureInitFrom(Stage::PARENT, {"group"});
    stage->setMinMaxDistance(.08, .13);

    geometry_msgs::Vector3Stamped vec;
    vec.header.frame_id = "s_model_tool0";
    vec.vector.z = -1.0;
    stage->setDirection(vec);
    t.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<stages::GeneratePose>("place pose");
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "table_top";
    p.pose.orientation.w = 1;
    p.pose.position.x = -0.15;
    p.pose.position.y = 0.35;
    p.pose.position.z = 0.15;
    stage->setPose(p);
    stage->properties().configureInitFrom(Stage::PARENT);

    stage->setMonitoredStage(object_grasped);

    auto wrapper = std::make_unique<stages::ComputeIK>("place pose kinematics",
                                                       std::move(stage));
    wrapper->setMaxIKSolutions(8);
    wrapper->setTimeout(0.5);
    // TODO: optionally in object frame
    wrapper->properties().configureInitFrom(
        Stage::PARENT, {"eef"}); // TODO: convenience wrapper
    wrapper->properties().configureInitFrom(Stage::INTERFACE, {"target_pose"});
    t.add(std::move(wrapper));
  }

  {
    auto stage =
        std::make_unique<stages::MoveTo>("release object", joint_interpolation);
    stage->properties().property("group").configureInitFrom(
        Stage::PARENT, "gripper"); // TODO this is not convenient
    stage->setGoal("open");
    t.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<stages::ModifyPlanningScene>(
        "forbid gripper->object collision");
    stage->allowCollisions("bottle",
                           t.getRobotModel()
                               ->getJointModelGroup("gripper")
                               ->getLinkModelNamesWithCollisionGeometry(),
                           false);
    t.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
    stage->detachObject("bottle", "s_model_tool0");
    object_grasped = stage.get();
    t.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<stages::MoveRelative>("retreat after place",
                                                        cartesian_planner);
    stage->properties().configureInitFrom(Stage::PARENT, {"group"});
    stage->setMinMaxDistance(.12, .25);
    stage->setIKFrame("s_model_tool0"); // TODO property for frame

    stage->setMarkerNS("post-place");

    geometry_msgs::Vector3Stamped vec;
    vec.header.frame_id = "s_model_tool0";
    vec.vector.x = -1.0;
    vec.vector.z = 0.75;
    stage->setDirection(vec);
    t.add(std::move(stage));
  }

  {
    auto stage =
        std::make_unique<stages::MoveTo>("move home", sampling_planner);
    stage->properties().configureInitFrom(Stage::PARENT, {"group"});
    stage->setGoal("pour_default");
    t.add(std::move(stage));
  }

  t.enableIntrospection();

  bool execute = pnh.param<bool>("execute", false);

  if (execute) {
    ROS_INFO("Going to execute best solution");
  }

  ROS_INFO_STREAM(t);

  // TODO: try { t.validate(); } catch() {}

  try {
    auto start_time { ros::WallTime::now() };
    t.plan(pnh.param<int>("solutions", 1));
    ROS_WARN_STREAM("Planning took "
	 << (ros::WallTime::now() - start_time).toSec() * 1000.0
	 << "ms to find "
	 << t.numSolutions() << " solution(s) with best solution "
	 << t.solutions().front()->cost()
	 );

  } catch (InitStageException &e) {
    ROS_ERROR_STREAM(e);
  }

  if (t.numSolutions() == 0) {
    ROS_ERROR("No solution found.");
  } else if(execute) {
    moveit_task_constructor_msgs::Solution solution;
    t.solutions().front()->toMsg(solution);
    std::cout << "executing solution" << std::endl;
    mtc_pour::executeSolution(solution);
    std::cout << "done" << std::endl;
  }

	// If wanted, keep introspection alive
	if (pnh.param("keep_running", true)){
		t.introspection();
		ros::waitForShutdown();
	}

  return 0;
}
