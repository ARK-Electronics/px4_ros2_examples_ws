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

class PrecisionLand : public px4_ros2::ModeBase
{
public:
	explicit PrecisionLand(rclcpp::Node& node);

	// Callbacks
	void vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);

	// See ModeBasep
	void onActivate() override;
	void onDeactivate() override;
	void updateSetpoint(float dt_s) override;

private:
	bool positionReached(const Eigen::Vector3f& target) const;

	enum class State {
		Home,
		Execute, 	// Executes the chosen pattern
		Descend, 	// Stay over landing target while descending
		Finished
	};

	void switchToState(State state);
	std::string stateName(State state);

	// ros2
	rclcpp::Node& _node;

	rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr _vehicle_land_detected_sub;

	// px4_ros2_cpp
	std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
	std::shared_ptr<px4_ros2::OdometryAttitude> _vehicle_attitude;
	std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;

	// Data
	State _state = State::Execute;
	Eigen::Vector3f _target_position = { NAN, NAN, NAN};
	float _approach_altitude = {};


	// Trajectory type parameter
	std::string _trajectory_type;
	// Waypoints for Search pattern
	std::vector<Eigen::Vector3f> _waypoints;
	// Search pattern generation using trajectory type
	void generateWaypoints();
	// Search pattern index
	int _waypoint_index = 0;
	// Land detection
	bool _land_detected = false;
};
