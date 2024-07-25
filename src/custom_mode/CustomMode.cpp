/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "CustomMode.hpp"

#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

static const std::string kModeName = "CustomMode";
static const bool kEnableDebugOutput = true;

using namespace px4_ros2::literals;

PrecisionLand::PrecisionLand(rclcpp::Node& node)
	: ModeBase(node, kModeName)
	, _node(node)
{

	_trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);

	_vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

	_vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);

	// Subscribe to vehicle_land_detected
	_vehicle_land_detected_sub = _node.create_subscription<px4_msgs::msg::VehicleLandDetected>("/fmu/out/vehicle_land_detected",
			   rclcpp::QoS(1).best_effort(), std::bind(&PrecisionLand::vehicleLandDetectedCallback, this, std::placeholders::_1));
	// Declare the trajectory type parameter with a default value of "circle"
	_node.declare_parameter("trajectory_type", "circle");
	_node.get_parameter("trajectory_type", _trajectory_type);
	
}

void PrecisionLand::vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
	_land_detected = msg->landed;
}

void PrecisionLand::onActivate()
{
	generateWaypoints();
	switchToState(State::Execute);
}

void PrecisionLand::onDeactivate()
{
	RCLCPP_INFO(_node.get_logger(), "Deactivating CustomMode");
}

void PrecisionLand::updateSetpoint(float dt_s)
{
	

	switch (_state) {
	case State::Home: {
		// Go home and land
		Eigen::Vector3f target_position = { 0.0f, 0.0f, _vehicle_local_position->positionNed().z()};

		px4_msgs::msg::TrajectorySetpoint setpoint;

		setpoint.timestamp = _node.now().nanoseconds() / 1000;
		setpoint.position = { target_position.x(), target_position.y(), target_position.z() };
		setpoint.velocity = { NAN, NAN, NAN };
		setpoint.acceleration = { NAN, NAN, NAN };
		setpoint.jerk = { NAN, NAN, NAN };
		setpoint.yaw = NAN;
		setpoint.yawspeed = NAN;
		// Publish the trajectory setpoint
		_trajectory_setpoint->update(setpoint);

		if (positionReached(target_position)) {
			switchToState(State::Descend);
		}

		break;

	}
	case State::Execute: {

		// Get the next waypoint
		Eigen::Vector3f target_position = _waypoints[_waypoint_index];

		px4_msgs::msg::TrajectorySetpoint setpoint;

		// Go to the next waypoint
		setpoint.timestamp = _node.now().nanoseconds() / 1000;
		setpoint.position = { target_position.x(), target_position.y(), target_position.z() };
		setpoint.velocity = { NAN, NAN, NAN };
		setpoint.acceleration = { NAN, NAN, NAN} ;
		setpoint.jerk = { NAN, NAN, NAN };
		setpoint.yaw = NAN;
		setpoint.yawspeed = NAN;
		// Publish the trajectory setpoint
		_trajectory_setpoint->update(setpoint);


		// Check if the drone has reached the target position
		if (positionReached(target_position)) {
			_waypoint_index++;

			// If we have Executeed all waypoints, go home
			if (_waypoint_index >= static_cast<int>(_waypoints.size())) {
				// Switch to state Home
				switchToState(State::Home);
			}
		}

		break;
	}

	

	case State::Descend: {

	
		px4_msgs::msg::TrajectorySetpoint setpoint;

		// This works
		setpoint.timestamp = _node.now().nanoseconds() / 1000;
		setpoint.position = {0.0, 0.0, NAN};
		setpoint.velocity = {NAN, NAN, 0.35};
		setpoint.acceleration = {NAN, NAN, NAN};
		setpoint.jerk = {NAN, NAN, NAN};
		setpoint.yaw = NAN;
		setpoint.yawspeed = NAN;

		//This does not work
		// setpoint.timestamp = _node.now().nanoseconds() / 1000;
		// setpoint.position = {NAN, NAN, NAN};
		// setpoint.velocity = {NAN, NAN, 0.35};
		// setpoint.acceleration = {NAN, NAN, NAN};
		// setpoint.jerk = {NAN, NAN, NAN};
		// setpoint.yaw = NAN;
		// setpoint.yawspeed = NAN;

	

		_trajectory_setpoint->update(setpoint);

		if (_land_detected) {
			switchToState(State::Finished);
		}

		break;
	}

	case State::Finished: {
		ModeBase::completed(px4_ros2::Result::Success);
		break;
	}
	} // end switch/case
}

void PrecisionLand::generateWaypoints()
{
	// Generate spiral Execute waypoints
	// The Execute waypoints are generated in the NED frame
	// Parameters for the Execute pattern
	double start_x = 0.0;
	double start_y = 0.0;
	double current_z = _vehicle_local_position->positionNed().z();
	

	double max_radius = 3.0;
	int points = 16;
	std::vector<Eigen::Vector3f> waypoints;

	// Generate waypoints

	// Generate waypoints
	// if the trajectory type is "spiral", generate spiral waypoints
	if (_trajectory_type == "spiral") {
		// Generate spiral waypoints
		// The spiral waypoints are generated in the NED frame
		// Parameters for the spiral pattern
		double radius = 0.0;

		for (int point = 0; point < points + 1; ++point) {
			double angle = 2.0 * M_PI * point / points;
			double x = start_x + radius * cos(angle);
			double y = start_y + radius * sin(angle);
			double z = current_z;

			waypoints.push_back(Eigen::Vector3f(x, y, z));
			radius += max_radius / points;
		}

		
	}
	// Trajectory type circle
	else if (_trajectory_type == "circle") {
		// Generate circle waypoints
		// The circle waypoints are generated in the NED frame
		// Parameters for the circle pattern
		double angle_increment = 2.0 * M_PI / points;

		for (int point = 0; point < points; ++point) {
			double angle = angle_increment * point;
			double x = start_x + max_radius * cos(angle);
			double y = start_y + max_radius * sin(angle);
			double z = current_z;

			waypoints.push_back(Eigen::Vector3f(x, y, z));
		}

	}
	// If trajectory type is figure 8
	else if (_trajectory_type == "figure_8") {
		// Generate figure 8 waypoints
		// The figure 8 waypoints are generated in the NED frame
		// Parameters for the figure 8 pattern
		    // Parameters for the figure 8 pattern
    	double angle_increment = 2.0 * M_PI / points;

    	for (int point = 0; point < points; ++point) {
			double angle = angle_increment * point;

			// Create the figure-8 pattern
			double x = start_x + max_radius * sin(angle);
			double y = start_y + max_radius * sin(2 * angle);
			double z = current_z;  // Assuming constant z

			waypoints.push_back(Eigen::Vector3f(x, y, z));
    }

		// Set the Execute waypoints
		_waypoints = waypoints;
	}

	// Set the Execute waypoints
	_waypoints = waypoints;
}

bool PrecisionLand::positionReached(const Eigen::Vector3f& target) const
{
	// TODO: parameters for delta_position and delta_velocitry
	static constexpr float kDeltaPosition = 0.25f;
	static constexpr float kDeltaVelocity = 0.25f;

	auto position = _vehicle_local_position->positionNed();
	auto velocity = _vehicle_local_position->velocityNed();

	const auto delta_pos = target - position;
	// NOTE: this does NOT handle a moving target!
	return (delta_pos.norm() < kDeltaPosition) && (velocity.norm() < kDeltaVelocity);
}

std::string PrecisionLand::stateName(State state)
{
	switch (state) {
	case State::Home:
		return "Home";
	case State::Execute:
		return "Execute";
	case State::Descend:
		return "Descend";
	case State::Finished:
		return "Finished";
	default:
		return "Unknown";
	}
}

void PrecisionLand::switchToState(State state)
{
	RCLCPP_INFO(_node.get_logger(), "Switching to %s", stateName(state).c_str());
	_state = state;
}

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<PrecisionLand>>(kModeName, kEnableDebugOutput));
	rclcpp::shutdown();
	return 0;
}
