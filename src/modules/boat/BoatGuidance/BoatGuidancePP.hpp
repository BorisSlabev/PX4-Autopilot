/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

// PX4 includes
#include <px4_platform_common/module_params.h>

// uORB includes
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
// #include <uORB/topics/ackermann_drive_setpoint.h>
#include <uORB/topics/boat_setpoint.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/position_controller_status.h>
#include <uORB/topics/mission_result.h>

// Standard library includes
#include <matrix/matrix/math.hpp>
#include <matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <math.h>

using namespace matrix;

/**
 * @brief Enum class for the different states of guidance.
 */
enum class GuidanceStatePP {
	kDriving, // The vehicle is currently driving.
	kGoalReached // The vehicle has reached its goal.
};

/**
 * @brief Class for ackermann drive guidance.
 */
class BoatGuidancePP : public ModuleParams
{
public:
	/**
	 * @brief Constructor for BoatGuidancePP.
	 * @param parent The parent ModuleParams object.
	 */
	BoatGuidancePP(ModuleParams *parent);
	~BoatGuidancePP() = default;

	/**
	 * @brief Compute guidance for ackermann rover.
	 */
	void purePursuit();

	/**
	 * @brief Return desired heading to the intersection point between a circle with a radius of lookahead_distance around the vehicle and a line segment
	 * from the previous to the current waypoint.
	 * @param curr_wp_local Current waypoint in local frame.
	 * @param prev_wp_local Previous waypoint in local frame.
	 * @param curr_pos_local Current position of the vehicle in local frame.
	 * @param lookahead_distance Radius of circle around vehicle.
	 */
	float calcDesiredHeading(const Vector2f &curr_wp_local, const Vector2f &prev_wp_local, const Vector2f &curr_pos_local,
				 const float &lookahead_distance);

	/**
	 * @brief Publishes the  acceptance radius for current waypoint based on the angle between a line segment
	 * from the previous to the current waypoint/current to the next waypoint and maximum steer angle of
	 * the vehicle.
	 * @param curr_wp_local Current waypoint in local frame.
	 * @param prev_wp_local Previous waypoint in local frame.
	 * @param next_wp_local Next waypoint in local frame.
	 */
	void updateAcceptanceRadius(Vector2f const &curr_wp_local, Vector2f const &prev_wp_local,
				    Vector2f const &next_wp_local);

	/**
	 * @brief Set the maximum speed for the vehicle.
	 * @param max_speed The maximum speed in m/s.
	 * @return The set maximum speed in m/s.
	 */
	float setMaxSpeed(float max_speed) { return _max_speed = max_speed; }


	/**
	 * @brief Set the maximum angular velocity for the vehicle.
	 * @param max_angular_velocity The maximum angular velocity in rad/s.
	 * @return The set maximum angular velocity in rad/s.
	 */
	float setMaxAngularVelocity(float max_angular_velocity) { return _max_angular_velocity = max_angular_velocity; }

protected:
	/**
	 * @brief Update the parameters of the module.
	 */
	void updateParams() override;

private:
	// uORB subscriptions
	uORB::Subscription _position_setpoint_triplet_sub{ORB_ID(position_setpoint_triplet)};
	uORB::Subscription _vehicle_global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _home_position_sub{ORB_ID(home_position)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _mission_result_sub{ORB_ID(mission_result)};
	uORB::Subscription _position_controller_status_sub{ORB_ID(position_controller_status)};
	position_setpoint_triplet_s _position_setpoint_triplet{};
	vehicle_local_position_s _local_position{};

	// uORB publications
	// uORB::Publication<ackermann_drive_setpoint_s> _ackermann_drive_setpoint_pub{ORB_ID(ackermann_drive_setpoint)};
	uORB::Publication<position_controller_status_s>	_pos_ctrl_status_pub{ORB_ID(position_controller_status)};
	uORB::Publication<boat_setpoint_s> _boat_setpoint_pub{ORB_ID(boat_setpoint)};
	// ackermann_drive_setpoint_s _ackermann_drive_setpoint{};
	boat_setpoint_s _boat_setpoint{};

	MapProjection _global_local_proj_ref{}; // Transform global to local coordinates.

	// Rover variables
	GuidanceStatePP _currentState = GuidanceStatePP::kDriving; // Current state of guidance.
	Vector2d _curr_pos{};
	Vector2f _curr_pos_local{};
	float _vehicle_yaw{0.f};
	float _desired_speed{0.f};
	float _desired_steering{0.f};

	// Waypoint variables
	Vector2d _curr_wp{};
	Vector2d _next_wp{};
	Vector2d _prev_wp{};
	Vector2f _curr_wp_local{};
	Vector2f _prev_wp_local{};
	Vector2f _next_wp_local{};
	float _prev_acc_rad{0.f};
	bool _mission_finished{false};

	float _max_speed{0.f};
	float _max_angular_velocity{0.f};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::AD_WHEEL_BASE>) _param_ad_wheel_base,
		(ParamFloat<px4::params::AD_MAX_STR_ANG>) _param_ad_max_steer_angle,
		(ParamFloat<px4::params::AD_LOOKAHEAD_TUN>) _param_ad_lookahead_tun,
		(ParamFloat<px4::params::AD_LOOKAHEAD_MAX>) _param_ad_lookahead_max,
		(ParamFloat<px4::params::AD_LOOKAHEAD_MIN>) _param_ad_lookahead_min,
		(ParamFloat<px4::params::AD_ACC_RAD_MAX>) _param_ad_acc_rad_max,
		(ParamFloat<px4::params::AD_ACC_RAD_DEF>) _param_ad_acc_rad_def,
		(ParamFloat<px4::params::AD_ACC_RAD_TUN>) _param_ad_acc_rad_tun,
		(ParamFloat<px4::params::AD_MISS_VEL_DEF>) _param_ad_miss_vel_def,
		(ParamFloat<px4::params::AD_MISS_VEL_MIN>) _param_ad_miss_vel_min,
		(ParamFloat<px4::params::AD_MISS_VEL_TUN>) _param_ad_miss_vel_tun,
		(ParamFloat<px4::params::NAV_ACC_RAD>) _param_nav_acc_rad
	)
};
