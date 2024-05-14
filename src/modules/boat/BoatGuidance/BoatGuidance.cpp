/****************************************************************************
 *
 *   Copyright (c) 2023-2024 PX4 Development Team. All rights reserved.
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

#include "BoatGuidance.hpp"

#include <mathlib/math/Limits.hpp>

using namespace matrix;

BoatGuidance::BoatGuidance(ModuleParams *parent) : ModuleParams(parent)
{
	updateParams();

	_currentState = GuidanceState::DRIVING;

	pid_init(&_heading_p_controller, PID_MODE_DERIVATIV_NONE, 0.001f);
}

void BoatGuidance::computeGuidance(float yaw, float angular_velocity, vehicle_local_position_s vehicle_local_position,
				   float dt)
{
	if (_position_setpoint_triplet_sub.updated()) {
		_position_setpoint_triplet_sub.copy(&_position_setpoint_triplet);
	}

	if (_vehicle_global_position_sub.updated()) {
		_vehicle_global_position_sub.copy(&_vehicle_global_position);
	}

	const matrix::Vector2d global_position(_vehicle_global_position.lat, _vehicle_global_position.lon);
	const matrix::Vector2d current_waypoint(_position_setpoint_triplet.current.lat, _position_setpoint_triplet.current.lon);
	const matrix::Vector2d next_waypoint(_position_setpoint_triplet.next.lat, _position_setpoint_triplet.next.lon);
	const matrix::Vector2d previous_waypoint(_position_setpoint_triplet.previous.lat,
			_position_setpoint_triplet.previous.lon);

	if (!_global_local_proj_ref.isInitialized()
	    || (_global_local_proj_ref.getProjectionReferenceTimestamp() != vehicle_local_position.timestamp)) {
		_global_local_proj_ref.initReference(vehicle_local_position.ref_lat, vehicle_local_position.ref_lon,
						     vehicle_local_position.timestamp);
	}

	const Vector2f current_waypoint_local_position = _global_local_proj_ref.project(current_waypoint(0),
			current_waypoint(1));
	// const Vector2f next_waypoint_local_position = _global_local_proj_ref.project(next_waypoint(0), next_waypoint(1));
	const Vector2f previous_waypoint_local_position = _global_local_proj_ref.project(previous_waypoint(0),
			previous_waypoint(1));
	const Vector2f local_position = Vector2f(vehicle_local_position.x, vehicle_local_position.y);
	const Vector2f local_velocity = Vector2f(vehicle_local_position.vx, vehicle_local_position.vy);

	const float distance_to_next_wp = get_distance_to_next_waypoint(local_position(0), local_position(1),
					  current_waypoint_local_position(0),
					  current_waypoint_local_position(1));

	float desired_heading = get_bearing_to_next_waypoint(global_position(0), global_position(1), next_waypoint(0),
				next_waypoint(1));
	float heading_error = matrix::wrap_pi(desired_heading - yaw);

	// printf("desired heading: %f\n", (double)desired_heading);
	// printf("yaw: %f\n", (double)yaw);
	// printf("heading error: %f\n", (double)heading_error);

	if (_current_waypoint != current_waypoint) {
		printf("switching back to driving\n");
		_currentState = GuidanceState::DRIVING;
	}

	// Make rover stop when it arrives at the last waypoint instead of loitering and driving around weirdly.
	if ((current_waypoint == next_waypoint) && distance_to_next_wp <= _param_nav_acc_rad.get()) {
		_currentState = GuidanceState::GOAL_REACHED;
	}

	_forwards_velocity_smoothing.updateDurations(_max_speed);
	_forwards_velocity_smoothing.updateTraj(dt);

	float speed_interpolation = 0.f;

	if (PX4_ISFINITE(heading_error)) {
		speed_interpolation = math::interpolate<float>(abs(heading_error), _param_bt_min_heading_error.get() * M_PI_F / 180.f,
				      _param_bt_max_heading_error.get() * M_PI_F / 180.f, _param_bt_spd_cruise.get(),
				      _param_bt_spd_min.get());
	}

	printf("speed interpolation: %f\n", (double)speed_interpolation);

	float desired_speed = math::constrain(speed_interpolation, _param_bt_spd_min.get(), _max_speed);

	printf("desired_speed: %f\n", (double)desired_speed);

	switch (_currentState) {
	case GuidanceState::DRIVING: {

			// _forwards_velocity_smoothing.updateDurations(_max_speed);
			// _forwards_velocity_smoothing.updateTraj(dt);

			// float speed_interpolation = math::interpolate<float>(abs(heading_error), _param_bt_min_heading_error.get(),
			// 			    _param_bt_max_heading_error.get(), _param_bt_spd_cruise.get(),
			// 			    _param_bt_spd_min.get());

			// desired_speed = math::constrain(speed_interpolation, _param_bt_spd_min.get(), _max_speed);

			if (PX4_ISFINITE(previous_waypoint(0)) && PX4_ISFINITE(previous_waypoint(1))) {
				_l1_guidance.navigate_waypoints(previous_waypoint_local_position, current_waypoint_local_position, local_position,
								local_velocity);
				printf("driving\n");

			} else {
				_previous_local_position = local_position;
				_currentState = GuidanceState::DRIVING_TO_POINT;
				printf("switched to driving to point\n");
			}

			// _desired_angular_velocity = math::constrain(_l1_guidance.nav_lateral_acceleration_demand(), -_max_angular_velocity,
			// 			    _max_angular_velocity);

			break;
		}

	case GuidanceState::DRIVING_TO_POINT:
		// _desired_angular_velocity = math::constrain(_l1_guidance.nav_lateral_acceleration_demand(), -_max_angular_velocity,
		// 			    _max_angular_velocity);
		printf("driving to point\n");
		printf("Previous local position: %f, %f\n", (double)_previous_local_position(0), (double)_previous_local_position(1));
		printf("Current waypoint local position: %f, %f\n", (double)current_waypoint_local_position(0),
		       (double)current_waypoint_local_position(1));
		_l1_guidance.navigate_waypoints(_previous_local_position, current_waypoint_local_position, local_position,
						local_velocity);
		desired_speed = _param_bt_spd_cruise.get();
		break;

	case GuidanceState::GOAL_REACHED:
		// temporary till I find a better way to stop the vehicle
		desired_speed = 0.f;
		heading_error = 0.f;
		angular_velocity = 0.f;
		_desired_angular_velocity = 0.f;
		printf("angular velocity: %f \n", (double)angular_velocity);
		break;
	}

	_desired_angular_velocity = math::constrain(_l1_guidance.nav_lateral_acceleration_demand(), -_max_angular_velocity,
				    _max_angular_velocity);

	// Compute the desired angular velocity relative to the heading error, to steer the vehicle towards the next waypoint.
	// float angular_velocity_pid = pid_calculate(&_heading_p_controller, heading_error, angular_velocity, 0,
	// 			     dt) + heading_error;

	// angular_velocity = angular_velocity + _desired_angular_velocity;

	printf("desired speed: %f\n", (double)desired_speed);
	printf("desired angular velocity: %f\n", (double)_desired_angular_velocity);
	printf("angular velocity: %f \n", (double)angular_velocity);

	boat_setpoint_s output{};
	output.speed = math::constrain(desired_speed, -_max_speed, _max_speed);
	output.yaw_rate = math::constrain(_desired_angular_velocity, -_max_angular_velocity, _max_angular_velocity);
	output.closed_loop_speed_control = output.closed_loop_yaw_rate_control = true;
	output.timestamp = hrt_absolute_time();

	_boat_setpoint_pub.publish(output);

	printf("Current waypoint (global): %f, %f\n", (double)_current_waypoint(0), (double)_current_waypoint(1));
	printf("Current waypoint (local): %f, %f\n", (double)current_waypoint(0), (double)current_waypoint(1));


	_current_waypoint = current_waypoint;
}

void BoatGuidance::updateParams()
{
	ModuleParams::updateParams();

	_l1_guidance.set_l1_damping(_param_bt_l1_damping.get());
	_l1_guidance.set_l1_period(_param_bt_l1_period.get());

	pid_set_parameters(&_heading_p_controller,
			   _param_rdd_p_gain_heading.get(),  // Proportional gain
			   0,  // Integral gain
			   0,  // Derivative gain
			   0,  // Integral limit
			   200);  // Output limit

	_forwards_velocity_smoothing.setMaxJerk(_param_rdd_max_jerk.get());
	_forwards_velocity_smoothing.setMaxAccel(_param_rdd_max_accel.get());
	_forwards_velocity_smoothing.setMaxVel(_max_speed);
}
