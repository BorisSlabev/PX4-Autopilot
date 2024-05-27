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

#include "BoatGuidancePP.hpp"

#include <mathlib/math/Limits.hpp>

using namespace matrix;

BoatGuidancePP::BoatGuidancePP(ModuleParams *parent) : ModuleParams(parent)
{
	updateParams();
}

void BoatGuidancePP::purePursuit()
{
	// uORB subscriber updates
	_local_position_sub.update(&_local_position);

	if (_mission_result_sub.updated()) {
		mission_result_s mission_result{};
		_mission_result_sub.copy(&mission_result);
		_mission_finished = mission_result.finished;
	}

	if (_position_setpoint_triplet_sub.updated()) {
		position_controller_status_s position_controller_status{};
		_position_controller_status_sub.update(&position_controller_status);
		_prev_acc_rad = position_controller_status.acceptance_radius;
		_position_setpoint_triplet_sub.copy(&_position_setpoint_triplet);
	}

	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);
		matrix::Quatf vehicle_attitude_quaternion = Quatf(vehicle_attitude.q);
		_vehicle_yaw = matrix::Eulerf(vehicle_attitude_quaternion).psi();
	}

	if (!_global_local_proj_ref.isInitialized()
	    || (_global_local_proj_ref.getProjectionReferenceTimestamp() != _local_position.ref_timestamp)) {
		_global_local_proj_ref.initReference(_local_position.ref_lat, _local_position.ref_lon, _local_position.ref_timestamp);
	}

	// Setup global frame variables
	if (_vehicle_global_position_sub.updated()) {
		vehicle_global_position_s vehicle_global_position{};
		_vehicle_global_position_sub.copy(&vehicle_global_position);
		_curr_pos = Vector2d(vehicle_global_position.lat, vehicle_global_position.lon);
	}

	_curr_wp = Vector2d(_position_setpoint_triplet.current.lat, _position_setpoint_triplet.current.lon);

	if (_position_setpoint_triplet.previous.valid) {
		_prev_wp(0) = _position_setpoint_triplet.previous.lat;
		_prev_wp(1) = _position_setpoint_triplet.previous.lon;

	} else {
		if (_home_position_sub.updated()) {
			home_position_s home_position{};
			_home_position_sub.copy(&home_position);
			_prev_wp = Vector2d(home_position.lat, home_position.lon);
		}
	}

	if (_position_setpoint_triplet.next.valid) {
		_next_wp(0) = _position_setpoint_triplet.next.lat;
		_next_wp(1) = _position_setpoint_triplet.next.lon;

	} else {
		_next_wp = _curr_wp;
	}

	// Setup local frame variables
	_curr_pos_local = Vector2f(_local_position.x, _local_position.y);
	_curr_wp_local = _global_local_proj_ref.project(_curr_wp(0), _curr_wp(1));
	_prev_wp_local = _global_local_proj_ref.project(_prev_wp(0), _prev_wp(1));
	_next_wp_local = _global_local_proj_ref.project(_next_wp(0), _next_wp(1));
	updateAcceptanceRadius(_curr_wp_local, _prev_wp_local, _next_wp_local);

	// Stop at final waypoint
	if (_mission_finished) {
		_currentState = GuidanceStatePP::kGoalReached;

	} else {
		_currentState = GuidanceStatePP::kDriving;
	}

	// Guidance logic
	switch (_currentState) {
	case GuidanceStatePP::kDriving: {
			// Set rover speed
			// Vector2f rover_velocity = {_local_position.vx, _local_position.vy};
			// _boat_setpoint.speed_actual = rover_velocity.norm();
			float distance_to_prev_wp = get_distance_to_next_waypoint(_curr_pos(0), _curr_pos(1),
						    _prev_wp(0),
						    _prev_wp(1));

			if (distance_to_prev_wp <= _prev_acc_rad) { // Cornering speed
				float cornering_speed = _param_ad_miss_vel_tun.get() / _prev_acc_rad;
				_desired_speed = math::constrain(cornering_speed, _param_ad_miss_vel_min.get(), _param_ad_miss_vel_def.get());

			} else { // Default mission speed
				_desired_speed = _param_ad_miss_vel_def.get();
			}

			// Calculate crosstrack error
			Vector2f prev_wp_to_curr_wp_local = _curr_wp_local - _prev_wp_local;
			Vector2f prev_wp_to_curr_pos_local = _curr_pos_local - _prev_wp_local;
			Vector2f distance_on_line_segment = ((prev_wp_to_curr_pos_local * prev_wp_to_curr_wp_local) /
							     prev_wp_to_curr_wp_local.length()) * prev_wp_to_curr_wp_local.normalized();
			Vector2f crosstrack_error = (_prev_wp_local + distance_on_line_segment) - _curr_pos_local;

			// Calculate desired heading towards lookahead point
			float desired_heading = 0.f;
			float lookahead_distance = math::constrain(_param_ad_lookahead_tun.get() * _desired_speed,
						   _param_ad_lookahead_min.get(), _param_ad_lookahead_max.get());

			if (crosstrack_error.longerThan(lookahead_distance)) {
				if (crosstrack_error.length() < _param_ad_lookahead_max.get()) {
					lookahead_distance = 1.1f * crosstrack_error.length(); // Scale lookahead radius
					desired_heading = calcDesiredHeading(_curr_wp_local, _prev_wp_local, _curr_pos_local, lookahead_distance);

				} else { // Excessively large crosstrack error
					desired_heading = calcDesiredHeading(_curr_wp_local, _curr_pos_local, _curr_pos_local, lookahead_distance);
				}

			} else { // Crosstrack error smaller than lookahead
				desired_heading = calcDesiredHeading(_curr_wp_local, _prev_wp_local, _curr_pos_local, lookahead_distance);
			}

			// Calculate desired steering to reach lookahead point
			float heading_error = matrix::wrap_pi(desired_heading - _vehicle_yaw);
			// _boat_setpoint.lookahead_distance = lookahead_distance; // For logging
			// _boat_setpoint.heading_error = (heading_error * 180.f) / (2 * M_PI_2_F); // For logging

			if (math::abs_t(heading_error) <= M_PI_2_F) {
				_desired_steering = atanf(2 * _param_ad_wheel_base.get() * sinf(heading_error) / lookahead_distance);

			} else if (heading_error > 0.f) {
				_desired_steering = atanf(2 * _param_ad_wheel_base.get() * (1.0f + sinf(heading_error - M_PI_2_F)) /
							  lookahead_distance);

			} else {
				_desired_steering = atanf(2 * _param_ad_wheel_base.get() * (-1.0f + sinf(heading_error + M_PI_2_F)) /
							  lookahead_distance);
			}

			_desired_steering = math::constrain(_desired_steering, -_param_ad_max_steer_angle.get(),
							    _param_ad_max_steer_angle.get());

			break;
		}

	case GuidanceStatePP::kGoalReached:
		// Stop the rover
		_desired_steering = 0.f;
		_desired_speed = 0.f;
		break;
	}

	printf("desired speed: %f\n", (double)_desired_speed);
	printf("desired steering: %f\n", (double)_desired_steering);

	boat_setpoint_s output{};
	output.speed = math::constrain(_desired_speed, -_max_speed, _max_speed);
	printf("max speed output %f\n", (double)output.speed);
	output.yaw_rate = math::constrain(_desired_steering*10.f, -_max_angular_velocity*10.f, _max_angular_velocity*10.f);
	printf("max angular output %f\n", (double)output.yaw_rate);
	output.closed_loop_speed_control = output.closed_loop_yaw_rate_control = true;
	output.timestamp = hrt_absolute_time();

	_boat_setpoint_pub.publish(output);

	// Publish ackermann drive setpoint
	// _boat_setpoint.speed = _desired_speed;
	// _boat_setpoint.steering = _desired_steering;
	// _boat_setpoint.manual = false;
	// _boat_setpoint.timestamp = hrt_absolute_time();
	// _ackermann_drive_setpoint_pub.publish(_boat_setpoint);
}

float BoatGuidancePP::calcDesiredHeading(const Vector2f &curr_wp_local, const Vector2f &prev_wp_local,
		Vector2f const &curr_pos_local,
		float const &lookahead_distance)
{
	// Setup variables
	float line_segment_slope = (curr_wp_local(1) - prev_wp_local(1)) / (curr_wp_local(0) - prev_wp_local(0));
	float line_segment_rover_offset = prev_wp_local(1) - curr_pos_local(1) + line_segment_slope * (curr_pos_local(
			0) - prev_wp_local(0));
	float a = -line_segment_slope;
	float c = -line_segment_rover_offset;
	float r = lookahead_distance;
	float x0 = -a * c / (a * a + 1.0f);
	float y0 = -c / (a * a + 1.0f);

	// Calculate intersection points
	if (c * c > r * r * (a * a + 1.0f) + FLT_EPSILON) { // No intersection points exist
		return 0.f;

	} else if (abs(c * c - r * r * (a * a + 1.0f)) < FLT_EPSILON) { // One intersection point exists
		return atan2f(y0, x0);

	} else { // Two intersetion points exist
		float d = r * r - c * c / (a * a + 1.0f);
		float mult = sqrt(d / (a * a + 1.0f));
		float ax, ay, bx, by;
		ax = x0 + mult;
		bx = x0 - mult;
		ay = y0 - a * mult;
		by = y0 + a * mult;
		Vector2f point1(ax, ay);
		Vector2f point2(bx, by);
		Vector2f distance1 = (curr_wp_local - curr_pos_local) - point1;
		Vector2f distance2 = (curr_wp_local - curr_pos_local) - point2;

		// Return intersection point closer to current waypoint
		if (distance1.norm_squared() < distance2.norm_squared()) {
			return atan2f(ay, ax);

		} else {
			return atan2f(by, bx);
		}
	}
}

void BoatGuidancePP::updateAcceptanceRadius(const Vector2f &curr_wp_local, const Vector2f &prev_wp_local,
		const Vector2f &next_wp_local)
{
	// Setup variables
	Vector2f curr_to_prev_wp_local = prev_wp_local - curr_wp_local;
	Vector2f curr_to_next_wp_local = next_wp_local - curr_wp_local;
	float acceptance_radius = _param_ad_acc_rad_def.get();

	// Calculate acceptance radius s.t. the rover cuts the corner tangential to the current and next line segment
	if (curr_to_next_wp_local.length() > 0.01f) {
		float theta = acosf((curr_to_prev_wp_local * curr_to_next_wp_local) / (curr_to_prev_wp_local.length() *
				    curr_to_next_wp_local.length())) / 2;
		float min_turning_radius = _param_ad_wheel_base.get() / sinf(_param_ad_max_steer_angle.get());
		float acceptance_radius_temp = min_turning_radius / tanf(theta);
		float acceptance_radius_temp_scaled = _param_ad_acc_rad_tun.get() *
						      acceptance_radius_temp; // Scale geometric ideal acceptance radius to account for kinematic and dynamic effects
		acceptance_radius = math::constrain<float>(acceptance_radius_temp_scaled, _param_ad_acc_rad_def.get(),
				    _param_ad_acc_rad_max.get());
	}

	// Publish updated acceptance radius
	position_controller_status_s pos_ctrl_status = {};
	pos_ctrl_status.acceptance_radius = acceptance_radius;
	pos_ctrl_status.timestamp = hrt_absolute_time();
	_pos_ctrl_status_pub.publish(pos_ctrl_status);
}

void BoatGuidancePP::updateParams()
{
	ModuleParams::updateParams();
}
