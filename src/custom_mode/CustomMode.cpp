/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "CustomMode.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/utils/geometry.hpp>

// Constants
static const std::string kModeName = "CustomMode";
static const bool kEnableDebugOutput = true;

using namespace px4_ros2::literals;

// Constructor for the CustomMode class
CustomMode::CustomMode(rclcpp::Node& node)
	: ModeBase(node, kModeName), _node(node)
{
	// Initialize the trajectory setpoint and odometry local position
	_trajectory_setpoint =
		std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
	_vehicle_local_position =
		std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

	// Subscribe to vehicle_land_detected
	_vehicle_land_detected_sub =_node.create_subscription<px4_msgs::msg::VehicleLandDetected>(
			"/fmu/out/vehicle_land_detected", rclcpp::QoS(1).best_effort(),
			std::bind(&CustomMode::vehicleLandDetectedCallback, this,
				  std::placeholders::_1));
	// Declare the trajectory type parameter with a default value of "circle"
	_node.declare_parameter("trajectory_type", "circle");
	_node.get_parameter("trajectory_type", _trajectory_type);
}

void CustomMode::vehicleLandDetectedCallback(
	const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
	_land_detected = msg->landed;
}

// OnActivate function generates waypoints and switches to the Execute state
void CustomMode::onActivate()
{
	generateWaypoints();
	switchToState(State::Execute);
}
// OnDeactivate function logs a message when the mode is deactivated
void CustomMode::onDeactivate()
{
	RCLCPP_INFO(_node.get_logger(), "Deactivating CustomMode");
}

// UpdateSetpoint function updates the setpoint based on the current state
void CustomMode::updateSetpoint(float dt_s)
{

	// Switching between states
	switch (_state) {
	// Execute state is the main state where the drone follows the waypoints
	case State::Execute: {

		// Get the next waypoint
		Eigen::Vector3f target_position = _waypoints[_waypoint_index];

		// Create a trajectory setpoint message
		px4_msgs::msg::TrajectorySetpoint setpoint;

		// Construct the trajectory setpoint message
		setpoint.timestamp = _node.now().nanoseconds() / 1000;
		setpoint.position = {target_position.x(), target_position.y(),
				     target_position.z()
				    };
		setpoint.velocity = {NAN, NAN, NAN};
		setpoint.acceleration = {NAN, NAN, NAN};
		setpoint.jerk = {NAN, NAN, NAN};
		setpoint.yaw = NAN;
		setpoint.yawspeed = NAN;

		// Publish the trajectory setpoint
		_trajectory_setpoint->update(setpoint);

		// Check if the drone has reached the target position
		if (positionReached(target_position)) {
			// Move to the next waypoint
			_waypoint_index++;

			// If we have Executeed all waypoints, go home
			if (_waypoint_index >= static_cast<int>(_waypoints.size())) {
				// Switch to state Home
				switchToState(State::Home);
			}
		}

		break;
	}

	// Home state is the state where the drone returns to the X,Y position where
	// the drone was armed
	case State::Home: {
		// Set the target position to the X,Y position where the drone was armed
		Eigen::Vector3f target_position = {
			0.0f, 0.0f, _vehicle_local_position->positionNed().z()
		};
		// Create a trajectory setpoint message
		px4_msgs::msg::TrajectorySetpoint setpoint;
		// Construct the trajectory setpoint message
		setpoint.timestamp = _node.now().nanoseconds() / 1000;
		setpoint.position = {target_position.x(), target_position.y(),
				     target_position.z()
				    };
		setpoint.velocity = {NAN, NAN, NAN};
		setpoint.acceleration = {NAN, NAN, NAN};
		setpoint.jerk = {NAN, NAN, NAN};
		setpoint.yaw = NAN;
		setpoint.yawspeed = NAN;
		// Publish the trajectory setpoint
		_trajectory_setpoint->update(setpoint);

		// Check if the drone has reached the target position
		if (positionReached(target_position)) {
			// Switch to state Descend
			switchToState(State::Descend);
		}

		break;
	}

	// Descend state is the state where the drone lands
	case State::Descend: {

		// Create a trajectory setpoint message
		px4_msgs::msg::TrajectorySetpoint setpoint;

		// Construct the trajectory setpoint message
		setpoint.timestamp = _node.now().nanoseconds() / 1000;
		setpoint.position = {0.0, 0.0, NAN};
		setpoint.velocity = {NAN, NAN, 0.35};
		setpoint.acceleration = {NAN, NAN, NAN};
		setpoint.jerk = {NAN, NAN, NAN};
		setpoint.yaw = NAN;
		setpoint.yawspeed = NAN;

		// Publish the trajectory setpoint
		_trajectory_setpoint->update(setpoint);

		// Check if the drone has landed
		if (_land_detected) {
			// Switch to state Finished
			switchToState(State::Finished);
		}

		break;
	}

	// Finished state is the state where the mode is completed
	case State::Finished: {
		ModeBase::completed(px4_ros2::Result::Success);
		break;
	}
	} // end switch/case
}

// GenerateWaypoints function generates waypoints based on the trajectory type
void CustomMode::generateWaypoints()
{
	// Set the start position
	double start_x = 0.0;
	double start_y = 0.0;
	double current_z = _vehicle_local_position->positionNed().z();

	// Set the maximum radius and number of points
	double max_radius = 3.0;
	int points = 16;
	// Create a vector to store the waypoints
	std::vector<Eigen::Vector3f> waypoints;

	// Trajectory type circle
	if (_trajectory_type == "circle") {
		// Parameters for the circle pattern
		double angle_increment = 2.0 * M_PI / points;

		// Generate circle waypoints
		// The circle waypoints are generated in the NED frame
		for (int point = 0; point < points; ++point) {
			double angle = angle_increment * point;
			double x = start_x + max_radius * cos(angle);
			double y = start_y + max_radius * sin(angle);
			double z = current_z;
			// Push the waypoints to the vector
			waypoints.push_back(Eigen::Vector3f(x, y, z));
		}

	}

	// Trajectory type spiral
	else if (_trajectory_type == "spiral") {

		// Parameters for the spiral pattern
		double radius = 0.0;

		// Generate spiral waypoints
		// The spiral waypoints are generated in the NED frame
		for (int point = 0; point < points + 1; ++point) {
			double angle = 2.0 * M_PI * point / points;
			double x = start_x + radius * cos(angle);
			double y = start_y + radius * sin(angle);
			double z = current_z;
			// Push the waypoints to the vector
			waypoints.push_back(Eigen::Vector3f(x, y, z));
			// Increase the radius
			radius += max_radius / points;
		}

	}

	// Trajectory type figure 8
	else if (_trajectory_type == "figure_8") {
		// Parameters for the figure 8 pattern
		double angle_increment = 2.0 * M_PI / points;

		// Generate figure 8 waypoints
		// The figure 8 waypoints are generated in the NED frame
		for (int point = 0; point < points; ++point) {
			double angle = angle_increment * point;

			// Create the figure-8 pattern
			double x = start_x + max_radius * sin(angle);
			double y = start_y + max_radius * sin(2 * angle);
			double z = current_z; // Assuming constant z
			// Push the waypoints to the vector
			waypoints.push_back(Eigen::Vector3f(x, y, z));
		}
	}

	// Set the Execute waypoints
	_waypoints = waypoints;
}
// PositionReached function checks if the drone has reached the target position
bool CustomMode::positionReached(const Eigen::Vector3f& target) const
{
	// Define the position and velocity thresholds
	static constexpr float kDeltaPosition = 0.25f;
	static constexpr float kDeltaVelocity = 0.25f;

	// Get the current position and velocity
	auto position = _vehicle_local_position->positionNed();
	auto velocity = _vehicle_local_position->velocityNed();

	// Calculate the delta position and velocity
	const auto delta_pos = target - position;

	return (delta_pos.norm() < kDeltaPosition) &&
	       (velocity.norm() < kDeltaVelocity);
}
// StateName function returns the name of the state
std::string CustomMode::stateName(State state)
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
// SwitchToState function switches to the specified state
void CustomMode::switchToState(State state)
{
	RCLCPP_INFO(_node.get_logger(), "Switching to %s", stateName(state).c_str());
	_state = state;
}

// Main function
int main(int argc, char* argv[])
{
	// Initialize the ROS2 node
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<CustomMode>>(
			     kModeName, kEnableDebugOutput));
	rclcpp::shutdown();
	return 0;
}
