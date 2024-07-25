#pragma once

// Include the necessary headers
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

class CustomMode : public px4_ros2::ModeBase
{
public:
	explicit CustomMode(rclcpp::Node& node);

	// Callback function for the vehicle_land_detected subscription
	void vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);

	// Called when the mode is activated
	void onActivate() override;
	// Called when the mode is deactivated
	void onDeactivate() override;
	// Serves as the "main loop" and is called at regular intervals. Runs the state machine.
	void updateSetpoint(float dt_s) override;

private:
	enum class State {
		ReturnToHome, // Returns to the X,Y position where the drone was armed
		Execute, // Executes the chosen pattern
		Descend, // Lands the drone
		Finished
	};

	bool positionReached(const Eigen::Vector3f& target) const;

	void switchToState(State state);

	std::string stateName(State state);

	// generates waypoints based on the trajectory type
	void generateWaypoints();

	rclcpp::Node& _node;

	// Subscription to the vehicle_land_detected topic
	rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr _vehicle_land_detected_sub;

	// PX4 ROS2 OdometryAttitude and OdometryLocalPosition instances
	std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
	std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;

	State _state = State::Execute;

	std::string _trajectory_type;
	std::vector<Eigen::Vector3f> _waypoints;
	int _waypoint_index = 0;

	bool _land_detected = false;
};
