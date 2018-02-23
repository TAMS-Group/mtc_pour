#include <ros/ros.h>

#include <moveit/robot_model/robot_model.h> // TODO shouldn't be necessary...

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/compute_ik.h>

#include "pour_into.h"

#include <moveit/task_constructor/stages/modify_planning_scene.h>

#include <moveit/task_constructor/stages/generate_grasp_pose.h>

#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

void spawnObjects(){
	moveit::planning_interface::PlanningSceneInterface psi;

	std::vector<moveit_msgs::CollisionObject> objects;

	moveit_msgs::CollisionObject o;
	o.id= "bottle";
	o.header.frame_id= "table_top";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x= -0.15;
	o.primitive_poses[0].position.y= 0.25;
	o.primitive_poses[0].position.z= 0.12;
	o.primitive_poses[0].orientation.w= 1.0;
	o.primitives.resize(1);
	o.primitives[0].type= shape_msgs::SolidPrimitive::CYLINDER;
	o.primitives[0].dimensions.resize(2);
	o.primitives[0].dimensions[0]= 0.23;
	o.primitives[0].dimensions[1]= 0.03;
	objects.push_back(o);

	o.id= "glass";
	o.primitive_poses[0].position.x= -0.1;
	o.primitive_poses[0].position.y= -0.01;
	o.primitive_poses[0].position.z= 0.06;
	o.primitives[0].dimensions[0]= 0.11;
	objects.push_back(o);

	psi.applyCollisionObjects(objects);
}

using namespace moveit::task_constructor;

int main(int argc, char** argv){
	ros::init(argc, argv, "mtc_pouring");

	ros::AsyncSpinner spinner(1);
	spinner.start();

	spawnObjects();

	// TODO: why does a restart trigger a new panel entry
	Task t;

   // TODO: id of solution in rviz panel is sometimes 0 and then changes

	auto sampling_planner = std::make_shared<solvers::PipelinePlanner>();
	// TODO: timeout is still 10 in move to pre-grasp
	sampling_planner->setTimeout(15.0);
	//pipeline->setPlannerId("");

	auto cartesian_planner = std::make_shared<solvers::CartesianPath>();

	t.setProperty("group", "arm");
	t.setProperty("eef", "gripper");
	t.setProperty("gripper", "gripper"); // TODO: use this

	Stage* current_state= nullptr;
	{
		auto _current_state = std::make_unique<stages::CurrentState>("current state");
		current_state= _current_state.get();
		t.add(std::move(_current_state));
	}

	{
		auto stage = std::make_unique<stages::MoveTo>("open gripper", sampling_planner);
		stage->setGroup("gripper");
		stage->setGoal("open");
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::Connect>("move to pre-grasp pose", sampling_planner);
		stage->properties().configureInitFrom(Stage::PARENT); // TODO: convenience-wrapper
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesian_planner);
		stage->properties().set("marker_ns", "approach"); // TODO: convenience wrapper
		stage->properties().set("link", "s_model_tool0");
		stage->properties().configureInitFrom(Stage::PARENT, {"group"});
		stage->setMinMaxDistance(.08, .15);

		geometry_msgs::Vector3Stamped vec;
		vec.header.frame_id = "s_model_tool0";
		vec.vector.x = 1.0;
		stage->along(vec);
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::GenerateGraspPose>("grasp work space pose");
		stage->properties().configureInitFrom(Stage::PARENT);
		stage->setGripperGraspPose("open");
		stage->setObject("bottle");
		stage->setToolToGraspTF(Eigen::Translation3d(0,0,0), "s_model_tool0");
		stage->setAngleDelta(M_PI/6);

		stage->setMonitoredStage(current_state);

		auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose", std::move(stage) );
		wrapper->setMaxIKSolutions(8);
		// TODO adding this will initialize "target_pose" which is internal (or isn't it?)
		//wrapper->properties().configureInitFrom(Stage::PARENT);
		wrapper->properties().configureInitFrom(Stage::PARENT, {"eef"}); // TODO: convenience wrapper
		t.add(std::move(wrapper));
	}

	// TODO: encapsulate these three states in stages::Grasp or similar
	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("allow gripper->object collision");
		stage->enableCollisions("bottle", t.getRobotModel()->getJointModelGroup("gripper")->getLinkModelNamesWithCollisionGeometry(), true);
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::MoveTo>("close gripper", sampling_planner);
		stage->properties().property("group").configureInitFrom(Stage::PARENT, "gripper"); // TODO this is not convenient
		stage->setGoal("closed");
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
		stage->attachObject("bottle", "s_model_tool0");
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::MoveRelative>("lift object", cartesian_planner);
		stage->properties().configureInitFrom(Stage::PARENT, {"group"});
		stage->setMinMaxDistance(.20,.3);
		stage->setLink("s_model_tool0"); // TODO property for frame

		stage->properties().set("marker_ns", "lift");

		geometry_msgs::Vector3Stamped vec;
		vec.header.frame_id= "table_top";
		vec.vector.z= 1.0;
		stage->along(vec);
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<mtc_pour::PourInto>("test pouring");
		stage->setBottle("bottle");
		stage->setContainer("glass");
		stage->setPourOffset(Eigen::Vector3d(0,0.01,0.03));
		stage->setTiltAngle(2.0);
		stage->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(stage));
	}

	t.enableIntrospection();

	ROS_INFO_STREAM( t );

	// TODO: t.validate();

	try {
		t.plan();

		std::cout << "waiting for <enter>" << std::endl;
		std::cin.get();
	}
	catch(InitStageException& e){
		ROS_ERROR_STREAM(e);
	}


	return 0;
}
