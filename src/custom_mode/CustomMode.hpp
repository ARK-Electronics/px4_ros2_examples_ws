/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>


#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <vector>

/**
 * @brief A custom mode class that demonstrates how to create a custom mode in PX4 ROS2
 * 
 */
class CustomMode : public px4_ros2::ModeBase
{
public:
	/**
	 * @brief Construct a new Custom Mode object
	 * 
	 * @param node The ROS2 node
	 */
	explicit CustomMode(rclcpp::Node& node);

	/**
	 * @brief Callback for the vehicle_land_detected topic
	 * 
	 * @param msg The message
	 */
	void vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);

	/**
	 * @brief Called when the mode is activated
	 * 
	 */
	void onActivate() override;

	/**
	 * @brief Called when the mode is deactivated
	 * 
	 */
	void onDeactivate() override;

	/**
	 * @brief Update the setpoint
	 * 
	 * @param dt_s The time step
	 */
	void updateSetpoint(float dt_s) override;

private:
	/**
	 * @brief Check if the drone has reached the target position
	 * 
	 * @param target The target position
	 * @return true if the drone has reached the target position
	 * @return false if the drone has not reached the target position
	 */
	bool positionReached(const Eigen::Vector3f& target) const;

	
	/**
	 * @brief Enum class for the state machine
	 * 
	 */
	enum class State {
		Home,		// Returns to the X,Y position where the drone was armed
		Execute, 	// Executes the chosen pattern
		Descend, 	// Lands the drone
		Finished
	};

	/**
	 * @brief Switch to a new state
	 * 
	 * @param state The new state
	 */
	void switchToState(State state);
	/**
	 * @brief Get the name of the state
	 * 
	 * @param state The state
	 * @return std::string The name of the state
	 */
	std::string stateName(State state);

	/**
	 * @brief The ROS2 node
	 * 
	 */
	rclcpp::Node& _node;

	/**
	 * @brief The subscription to the vehicle_land_detected topic
	 * 
	 */
	rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr _vehicle_land_detected_sub;

	/**
	 * @brief The vehicle local position
	 * 
	 */
	std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
	/**
	 * @brief The trajectory setpoint
	 * 
	 */
	std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;

	/**
	 * @brief The current state
	 * 
	 */
	State _state = State::Execute;

	/**
	 * @brief Trajectory type parameter
	 * 
	 */
	std::string _trajectory_type;
	/**
	 * @brief Waypoints for the search pattern
	 * 
	 */
	std::vector<Eigen::Vector3f> _waypoints;
	/**
	 * @brief Generate waypoints for the pattern
	 * 
	 */
	void generateWaypoints();
	/**
	 * @brief The current waypoint index
	 * 
	 */
	int _waypoint_index = 0;
	/**
	 * @brief The landing status
	 * 
	 */
	bool _land_detected = false;
};
