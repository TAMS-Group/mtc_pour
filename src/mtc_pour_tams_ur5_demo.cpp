#include <ros/ros.h>

#include <moveit/robot_model/robot_model.h> // TODO shouldn't be necessary...

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/compute_ik.h>

#include <moveit/task_constructor/stages/pour_into.h>

#include <moveit/task_constructor/stages/modify_planning_scene.h>

#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_pose.h>

#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

void spawnObjects(){
	moveit::planning_interface::PlanningSceneInterface psi;

	std::vector<moveit_msgs::CollisionObject> objects;

	moveit_msgs::CollisionObject o;
	o.id= "bottle";
	o.header.frame_id= "table_top";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x= 0.0;
	o.primitive_poses[0].position.y= 0.15;
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
	o.primitive_poses[0].position.y= -0.12;
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
	// TODO: ignored because it is always overruled by Connect's timeout property
	//sampling_planner->setTimeout(15.0);
	//pipeline->setPlannerId("");

	// don't spill liquid
	moveit_msgs::Constraints upright_constraint;
	//upright_constraint.name = "s_model_tool0:upright:20000:high";
	//upright_constraint.orientation_constraints.resize(1);
	//{
	//	moveit_msgs::OrientationConstraint& c= upright_constraint.orientation_constraints[0];
	//	c.link_name= "s_model_tool0";
	//	c.header.frame_id= "table_top";
	//	c.orientation.w= 1.0;
	//	c.absolute_x_axis_tolerance= 0.65;
	//	c.absolute_y_axis_tolerance= 0.65;
	//	c.absolute_z_axis_tolerance= M_PI;
	//	c.weight= 1.0;
	//}

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
		stage->setMinMaxDistance(.15, .25);

		geometry_msgs::Vector3Stamped vec;
		vec.header.frame_id = "s_model_tool0";
		vec.vector.x = 1.0;
		stage->along(vec);
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::GenerateGraspPose>("grasp work space pose");
		stage->properties().configureInitFrom(Stage::PARENT);
		stage->setNamedPose("open");
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

	Stage* object_grasped= nullptr;
	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
		stage->attachObject("bottle", "s_model_tool0");
		object_grasped= stage.get();
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::MoveRelative>("lift object", cartesian_planner);
		stage->properties().configureInitFrom(Stage::PARENT, {"group"});
		stage->setMinMaxDistance(.08,.13);
		stage->setLink("s_model_tool0"); // TODO property for frame

		stage->properties().set("marker_ns", "lift");

		geometry_msgs::Vector3Stamped vec;
		vec.header.frame_id= "table_top";
		vec.vector.z= 1.0;
		stage->along(vec);
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::Connect>("move to pre-pour pose", sampling_planner);
		stage->setTimeout(ros::Duration(20.0));
		stage->setPathConstraints(upright_constraint);
		stage->properties().configureInitFrom(Stage::PARENT); // TODO: convenience-wrapper
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::GeneratePose>("pose above glass");
		geometry_msgs::PoseStamped p;
		p.header.frame_id= "glass";
		p.pose.orientation.w= 1;
		p.pose.position.z= .3;
		stage->setPose(p);
		stage->properties().configureInitFrom(Stage::PARENT);

		stage->setMonitoredStage(object_grasped);

		auto wrapper = std::make_unique<stages::ComputeIK>("pre-pour pose", std::move(stage) );
		wrapper->setMaxIKSolutions(8);
		// TODO adding this will initialize "target_pose" which is internal (or isn't it?)
		//wrapper->properties().configureInitFrom(Stage::PARENT);
		wrapper->properties().configureInitFrom(Stage::PARENT, {"eef"}); // TODO: convenience wrapper
		t.add(std::move(wrapper));
	}

	{
		auto stage = std::make_unique<mtc_pour::PourInto>("pouring");
		stage->setBottle("bottle");
		stage->setContainer("glass");
		stage->setPourOffset(Eigen::Vector3d(0,0.01,0.03));
		stage->setTiltAngle(2.0);
		stage->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(stage));
	}

	// PLACE

	{
		auto stage = std::make_unique<stages::Connect>("move to pre-place pose", sampling_planner);
		stage->setTimeout(ros::Duration(20.0));
		stage->setPathConstraints(upright_constraint);
		stage->properties().configureInitFrom(Stage::PARENT); // TODO: convenience-wrapper
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::MoveRelative>("put down object", cartesian_planner);
		stage->properties().set("marker_ns", "approach_place"); // TODO: convenience wrapper
		stage->properties().set("link", "s_model_tool0");
		stage->properties().configureInitFrom(Stage::PARENT, {"group"});
		stage->setMinMaxDistance(.08, .13);

		geometry_msgs::Vector3Stamped vec;
		vec.header.frame_id = "s_model_tool0";
		vec.vector.z = -1.0;
		stage->along(vec);
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::GeneratePose>("place pose");
		geometry_msgs::PoseStamped p;
		p.header.frame_id= "table_top";
		p.pose.orientation.w= 1;
		p.pose.position.x= -0.15;
		p.pose.position.y=  0.35;
		p.pose.position.z=  0.125;
		stage->setPose(p);
		stage->properties().configureInitFrom(Stage::PARENT);

		stage->setMonitoredStage(object_grasped);

		auto wrapper = std::make_unique<stages::ComputeIK>("place pose kinematics", std::move(stage) );
		wrapper->setMaxIKSolutions(8);
		// TODO: optionally in object frame
		wrapper->properties().configureInitFrom(Stage::PARENT, {"eef"}); // TODO: convenience wrapper
		t.add(std::move(wrapper));
	}

	{
		auto stage = std::make_unique<stages::MoveTo>("release object", sampling_planner);
		stage->properties().property("group").configureInitFrom(Stage::PARENT, "gripper"); // TODO this is not convenient
		stage->setGoal("open");
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("allow gripper->object collision");
		stage->enableCollisions("bottle", t.getRobotModel()->getJointModelGroup("gripper")->getLinkModelNamesWithCollisionGeometry(), false);
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
		stage->detachObject("bottle", "s_model_tool0");
		object_grasped= stage.get();
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::MoveRelative>("retreat after place", cartesian_planner);
		stage->properties().configureInitFrom(Stage::PARENT, {"group"});
		stage->setMinMaxDistance(.15,.25);
		stage->setLink("s_model_tool0"); // TODO property for frame

		stage->properties().set("marker_ns", "post-place");

		geometry_msgs::Vector3Stamped vec;
		vec.header.frame_id= "s_model_tool0";
		vec.vector.x= -1.0;
		vec.vector.z= 0.75;
		stage->along(vec);
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::MoveTo>("move home", sampling_planner);
		stage->properties().configureInitFrom(Stage::PARENT, {"group"});
		stage->setGoal("home");
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
