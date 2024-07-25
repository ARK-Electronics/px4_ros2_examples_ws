#pragma once

// Include the necessary headers
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

// CustomMode class definition that inherits from px4_ros2::ModeBase
class CustomMode : public px4_ros2::ModeBase
{
public:
	// Constructor that takes a reference to a rclcpp::Node
	explicit CustomMode(rclcpp::Node& node);

	// Callback function for the vehicle_land_detected subscription
	void vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);

	// Activates the mode
	void onActivate() override;
	// Deactivates the mode
	void onDeactivate() override;
	// serves as the "main loop" and is called at regular intervals, runs the state machine
	void updateSetpoint(float dt_s) override;

private:
	// Enum class for the different states of the state machine
	enum class State {
		ReturnToHome,    // Returns to the X,Y position where the drone was armed
		Execute, // Executes the chosen pattern
		Descend, // Lands the drone
		Finished
	};

	// Checks if the drone has reached the target position
	bool positionReached(const Eigen::Vector3f& target) const;

	// Switches to the given state
	void switchToState(State state);

	// Returns the name of the given state
	std::string stateName(State state);

	// Waypoint generation function
	void generateWaypoints();

	// ROS2 node
	rclcpp::Node& _node;

	// Subscription to the vehicle_land_detected topic
	rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr _vehicle_land_detected_sub;

	// PX4 ROS2 OdometryAttitude and OdometryLocalPosition instances
	std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
	std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;

	// Current state of the state machine
	State _state = State::Execute;

	// Trajectory type parameter
	std::string _trajectory_type;
	// Waypoints for the drone to follow
	std::vector<Eigen::Vector3f> _waypoints;

	// Waypoint index
	int _waypoint_index = 0;
	// Land detected flag
	bool _land_detected = false;
};
