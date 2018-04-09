#include <ros/ros.h>

#include <moveit/robot_model/robot_model.h> // TODO shouldn't be necessary...

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_task_constructor_msgs/Solution.h>

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

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shape_extents.h>

#include <shape_msgs/Mesh.h>

using namespace moveit::task_constructor;

ros::Subscriber solution_listener;

void monitorSolution(const moveit_task_constructor_msgs::Solution& solution){
	// we are only looking for one solution
	solution_listener.shutdown();

	ROS_INFO("Received first solution. Executing.");

	moveit::planning_interface::PlanningSceneInterface psi;
	moveit::planning_interface::MoveGroupInterface mgi("arm");

	ros::Duration(1.0).sleep();

	moveit::planning_interface::MoveGroupInterface::Plan plan;

	for(const moveit_task_constructor_msgs::SubTrajectory& traj : solution.sub_trajectory){
		if( traj.trajectory.joint_trajectory.points.empty() ){
			ROS_INFO("skipping empty trajectory");
		}
		else {
			ROS_INFO_STREAM("executing subtrajectory " << traj.id);
			plan.trajectory_= traj.trajectory;
			if(!static_cast<bool>(mgi.execute(plan))){
				ROS_ERROR("Execution failed! Aborting!");
				ros::Duration(5.0).sleep();
				mgi.setNamedTarget("home");
				mgi.move();
				ros::shutdown();
				return;
			}
		}
		psi.applyPlanningScene(traj.scene_diff);
	}

	ROS_INFO("Executed successfully.");
	ros::shutdown();
}

namespace {
	void importMeshFromResource(shape_msgs::Mesh& mesh_msg, const std::string& resource){
		const Eigen::Vector3d scaling(1, 1, 1);
		shapes::Shape* shape = shapes::createMeshFromResource(resource, scaling);
		shapes::ShapeMsg shape_msg;
		shapes::constructMsgFromShape(shape, shape_msg);
		mesh_msg = boost::get<shape_msgs::Mesh>(shape_msg);
	}

	void collisionObjectFromResource(moveit_msgs::CollisionObject& msg, const std::string& id, const Eigen::Vector3d& pose, const std::string& resource)
	{
		msg.meshes.resize(1);
		importMeshFromResource(msg.meshes[0], resource);

		msg.mesh_poses.resize(1);
		msg.mesh_poses[0].orientation.w = 1.0;
		msg.mesh_poses[0].position.x = pose[0];
		msg.mesh_poses[0].position.y = pose[1];
		msg.mesh_poses[0].position.z = pose[2];

		msg.header.frame_id = "table_top";
		msg.id = id;
		msg.operation = moveit_msgs::CollisionObject::ADD;
	}

	double computeMeshHeight(const shape_msgs::Mesh& mesh){
		double x,y,z;
		geometric_shapes::getShapeExtents(mesh, x, y, z);
		return z;
	}
}

void setupObjects(){
	moveit::planning_interface::PlanningSceneInterface psi;

	std::vector<moveit_msgs::CollisionObject> objects;

	{
		auto attached_objects = psi.getAttachedObjects({"bottle"});
		if(attached_objects.count("bottle") > 0){
			attached_objects["bottle"].object.operation = moveit_msgs::CollisionObject::REMOVE;
			psi.applyAttachedCollisionObject(attached_objects["bottle"]);
		}
	}

	objects.emplace_back();
	collisionObjectFromResource(
		objects.back(),
		"bottle",
		Eigen::Vector3d(0.0, 0.15, 0.0),
		"package://mtc_pour/meshes/bottle.stl");
	// should be *on* the table
	objects.back().mesh_poses[0].position.z+= computeMeshHeight(objects.back().meshes[0])/2 + .002;

	objects.emplace_back();
	collisionObjectFromResource(
		objects.back(),
		"glass",
		Eigen::Vector3d(-.1, -.12, 0.0),
		"package://mtc_pour/meshes/glass.stl");
	// should be *on* the table
	objects.back().mesh_poses[0].position.z+= computeMeshHeight(objects.back().meshes[0])/2 + .002;

	psi.applyCollisionObjects(objects);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "mtc_pouring");

	ros::AsyncSpinner spinner(2);
	spinner.start();

	setupObjects();

	// TODO: why does a restart trigger a new panel entry
	Task t;

	// TODO: id of solution in rviz panel is sometimes 0 and then changes

	auto sampling_planner = std::make_shared<solvers::PipelinePlanner>();
	// TODO: ignored because it is always overruled by Connect's timeout property
	//sampling_planner->setTimeout(15.0);
	//pipeline->setPlannerId("");

	// don't spill liquid
	moveit_msgs::Constraints upright_constraint;
	upright_constraint.name = "s_model_tool0:upright:20000:high";
	upright_constraint.orientation_constraints.resize(1);
	{
		moveit_msgs::OrientationConstraint& c= upright_constraint.orientation_constraints[0];
		c.link_name= "s_model_tool0";
		c.header.frame_id= "table_top";
		c.orientation.w= 1.0;
		c.absolute_x_axis_tolerance= 0.65;
		c.absolute_y_axis_tolerance= 0.65;
		c.absolute_z_axis_tolerance= M_PI;
		c.weight= 1.0;
	}

	auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
	cartesian_planner->setMaxVelocityScaling(.3);
	cartesian_planner->setMaxAccelerationScaling(.3);
	cartesian_planner->setStepSize(.002);

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
		stage->setAngleDelta(M_PI/6);

		stage->setMonitoredStage(current_state);

		auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose", std::move(stage) );
		wrapper->setMaxIKSolutions(8);
		wrapper->setIKFrame(Eigen::Translation3d(0.05,0,0), "s_model_tool0");
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
		stage->setPourOffset(Eigen::Vector3d(0,0.015,0.035));
		stage->setTiltAngle(2.0);
		stage->setPourDuration(ros::Duration(4.0));
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
		p.pose.position.z=  0.15;
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
		stage->setGoal("pour_default");
		t.add(std::move(stage));
	}

	t.enableIntrospection();

	ros::NodeHandle nh("~");

	bool execute= nh.param<bool>("execute", false);

	if(execute){
		ROS_INFO("Going to execute first computed solution");
		// TODO: node-internal callback on solution objects
		solution_listener= nh.subscribe("solution", 1, &monitorSolution);
	}

	ROS_INFO_STREAM( t );

	// TODO: try { t.validate(); } catch() {}

	try {
		// TODO: optionally also plan stages if incoming states have infinite cost. This facilitates debugging
		t.plan();

		if(!execute){
			std::cout << "waiting for <enter>" << std::endl;
			std::cin.get();
		}
		else {
			ros::waitForShutdown();
		}
	}
	catch(InitStageException& e){
		ROS_ERROR_STREAM(e);
	}


	return 0;
}
