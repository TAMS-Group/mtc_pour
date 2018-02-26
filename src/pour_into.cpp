/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017-2018, Hamburg University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Authors: Michael Goerner, Henning Kayser
   Desc:    Pour from attached bottle into(onto) an object
*/

#include "pour_into.h"

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <shape_msgs/SolidPrimitive.h>

#include <eigen_conversions/eigen_msg.h>

#include <rviz_marker_tools/marker_creation.h>

namespace mtc_pour {

PourInto::PourInto(std::string name) :
	PropagatingForward(std::move(name))
{
	auto& p = properties();
	p.declare<std::string>("group", "name of planning group");

	p.declare<std::string>("bottle", "attached bottle-like object");
	p.declare<std::string>("container", "container object to be filled");

	p.declare<double>("tilt_angle", "maximum tilt-angle for the bottle");
	p.declare<Eigen::Vector3d>("pour_offset", "offset for the bottle tip w.r.t. container top-center during pouring");
	p.declare<ros::Duration>("pour_duration", ros::Duration(1.0), "duration to stay in pouring pose");
}

bool PourInto::computeForward(const InterfaceState& from) {
	planning_scene::PlanningScenePtr to;
	SubTrajectory trajectory;

	bool success = !compute(from, to, trajectory);
	sendForward(from, InterfaceState(to), std::move(trajectory));
	return success;
}

namespace {
/** Compute prototype waypoints for pouring.
  * This generates a trajectory that pours in the Y-Z axis.
  * The generated poses are for the bottle-tip and are relative to the container top-center */
void computePouringWaypoints(const Eigen::Affine3d& start_tip_pose, double tilt_angle,
                             const Eigen::Translation3d& pouring_offset, EigenSTL::vector_Affine3d& waypoints,
                             unsigned int nr_of_waypoints= 10) {
	Eigen::Affine3d tip_pose(start_tip_pose);

	waypoints.push_back(start_tip_pose);

	for(unsigned int i= 1; i <= nr_of_waypoints; ++i){
		const double fraction= (double) i / nr_of_waypoints;
		const double exp_fraction= fraction*fraction;

		// linear interpolation for tilt angle
		Eigen::AngleAxisd rotation(fraction*tilt_angle, Eigen::Vector3d::UnitX());

		// exponential interpolation towards container rim + offset
		Eigen::Translation3d translation(
			start_tip_pose.translation()     * (1-exp_fraction) +
			pouring_offset.translation() * exp_fraction
			);

		waypoints.push_back(translation*rotation);
	}
}
}

bool PourInto::compute(const InterfaceState& input, planning_scene::PlanningScenePtr& result, SubTrajectory& trajectory) {
	const auto& props= properties();

	const std::string& container_name= props.get<std::string>("container");
	const std::string& bottle_name= props.get<std::string>("bottle");

	const Eigen::Translation3d pour_offset( props.get<Eigen::Vector3d>("pour_offset") );
	const auto& tilt_angle= props.get<double>("tilt_angle");

	const ros::Duration pour_duration( props.get<ros::Duration>("pour_duration") );

	const planning_scene::PlanningScene& scene= *input.scene();
	moveit::core::RobotModelConstPtr robot_model= scene.getRobotModel();
	const moveit::core::JointModelGroup* group= robot_model->getJointModelGroup(props.get<std::string>("group"));

	moveit_msgs::CollisionObject container;
	if(!scene.getCollisionObjectMsg(container, container_name))
		throw std::runtime_error("container object '" + container_name + "' is not specified in input planning scene");
	if(container.primitives[0].type != shape_msgs::SolidPrimitive::CYLINDER)
		throw std::runtime_error("PourInto expects container to be a cylinder");

	moveit_msgs::AttachedCollisionObject bottle;
	if(!scene.getAttachedCollisionObjectMsg(bottle, bottle_name))
		throw std::runtime_error("bottle '" + bottle_name + "' is not an attached collision object in input planning scene");
	if(bottle.object.primitives[0].type != shape_msgs::SolidPrimitive::CYLINDER)
		throw std::runtime_error("PourInto expects bottle object to have a cylinder as bottle tip as first primitive");

	moveit::core::RobotState state(scene.getCurrentState());

	// container frame: top-center of container object
   const Eigen::Affine3d& container_frame=
		scene.getFrameTransform(container_name) *
		Eigen::Translation3d(Eigen::Vector3d(0,0,container.primitives[0].dimensions[0]/2));

	const Eigen::Affine3d& bottle_frame= scene.getFrameTransform(bottle_name);

	// assume bottle tip as top-center of cylinder
	const Eigen::Translation3d bottle_tip(Eigen::Vector3d(0, 0, bottle.object.primitives[0].dimensions[0]/2));

	auto& attached_bottle_tfs= state.getAttachedBody(bottle_name)->getFixedTransforms();

	assert(attached_bottle_tfs.size() > 0 && "impossible: attached body does not know transform to its link");
	const Eigen::Affine3d bottle_tip_in_tool_link(attached_bottle_tfs[0]*bottle_tip);

	const Eigen::Affine3d bottle_tip_in_container_frame=
		container_frame.inverse() *
		bottle_frame *
		bottle_tip;

	EigenSTL::vector_Affine3d waypoints;
	computePouringWaypoints(bottle_tip_in_container_frame, tilt_angle, pour_offset, waypoints);

	// TODO: possibly also spawn alternatives:
	//for(auto& waypoint : waypoints)
	//	waypoint= Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()) * waypoint * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());

	for(auto& waypoint : waypoints)
		waypoint= container_frame*waypoint;

	for(auto waypoint : waypoints){
		geometry_msgs::PoseStamped p;
		p.header.frame_id= scene.getPlanningFrame();
		tf::poseEigenToMsg(waypoint, p.pose);

		visualization_msgs::Marker tip;
		tip.ns= "pouring waypoints";
		tip.header= p.header;
		tip.pose= rviz_marker_tools::composePoses(p.pose, Eigen::Affine3d(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d(0,1,0))));
		tip.color.r= .588;
		tip.color.g= .196;
		tip.color.b= .588;
		tip.color.a= 1.0;
		// TODO: rename or move this package! maybe move it in with moveit_visual_tools?
		rviz_marker_tools::makeArrow(tip, .11, true);
		trajectory.markers().push_back(tip);
	}

	for(auto& waypoint : waypoints)
		waypoint= waypoint*bottle_tip_in_tool_link.inverse();

	std::vector<moveit::core::RobotStatePtr> traj; // TODO: multi-waypoint callback in cartesian_planner?

	double path_fraction= state.computeCartesianPath(
		group,
		traj,
		state.getLinkModel(bottle.link_name),
		waypoints,
		true /* global reference_frame */,
		.02 /* max step size */,
		2.0 /* jump threshold */,
		[&scene](moveit::core::RobotState* rs,
		         const moveit::core::JointModelGroup* jmg,
		         const double* joint_positions){
			rs->setJointGroupPositions(jmg, joint_positions);
			rs->update();
			return !scene.isStateColliding(*rs, jmg->getName());
			}
		);

	auto robot_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model, group);
	for(const auto& waypoint : traj){
		robot_trajectory->addSuffixWayPoint(waypoint, 0.0);
	}

	robot_trajectory::RobotTrajectory back_trajectory(*robot_trajectory);
	back_trajectory.reverse();

	trajectory_processing::IterativeParabolicTimeParameterization iptp;
	iptp.computeTimeStamps(*robot_trajectory);
	iptp.computeTimeStamps(back_trajectory);

	robot_trajectory->append(back_trajectory, pour_duration.toSec());

	trajectory.setTrajectory(robot_trajectory);

	result= scene.diff();
	result->setCurrentState(robot_trajectory->getLastWayPoint());

	if ( path_fraction < 1.0 - .1 ){
		ROS_WARN_STREAM("PourInto only produced motion for " << path_fraction << " of the way. Rendering invalid");
		trajectory.setCost(std::numeric_limits<double>::infinity());
		return false;
	}

	return true;
}

}
