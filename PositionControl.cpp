/****************************************************************************
 *
 *   Copyright (c) 2018 - 2019 PX4 Development Team. All rights reserved.
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

/**
 * @file PositionControl.cpp
 */

#include "PositionControl.hpp"
#include "ControlMath.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <ecl/geo/geo.h>

using namespace matrix;

void PositionControl::setVelocityGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_vel_p = P;
	_gain_vel_i = I;
	_gain_vel_d = D;
}

void PositionControl::setVelocityLimits(const float vel_horizontal, const float vel_up, const float vel_down)
{
	_lim_vel_horizontal = vel_horizontal;
	_lim_vel_up = vel_up;
	_lim_vel_down = vel_down;
}

void PositionControl::setThrustLimits(const float min, const float max)
{
	// make sure there's always enough thrust vector length to infer the attitude
	_lim_thr_min = math::max(min, 10e-4f);
	_lim_thr_max = max;
}

void PositionControl::updateHoverThrust(const float hover_thrust_new)
{
	_vel_int(2) += (hover_thrust_new - _hover_thrust) * (CONSTANTS_ONE_G / hover_thrust_new);
	setHoverThrust(hover_thrust_new);
}

void PositionControl::setState(const PositionControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_yaw = states.yaw;
	_vel_dot = states.acceleration;
}

void PositionControl::setInputSetpoint(const vehicle_local_position_setpoint_s &setpoint)
{
	_pos_sp = Vector3f(setpoint.x, setpoint.y, setpoint.z);
	_vel_sp = Vector3f(setpoint.vx, setpoint.vy, setpoint.vz);
	_acc_sp = Vector3f(setpoint.acceleration);
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;
}

bool PositionControl::update(const float dt)
{
	// x and y input setpoints always have to come in pairs
	const bool valid = (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1)))
			   && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1)))
			   && (PX4_ISFINITE(_acc_sp(0)) == PX4_ISFINITE(_acc_sp(1)));

	_positionControl();
	_velocityControl(dt);
	// _positionSMC();

	_yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
	_yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw; // TODO: better way to disable yaw control

	return valid && _updateSuccessful();
}



// void PositionControl::_positionSMC()
// {
// 	// Sliding surface
// 	Vector3f vel_error = _vel_sp - _vel;
// 	Vector3f surface = vel_error + (_pos_sp - _pos).emult(_gain_pos_p);
// 	float s_x = surface(0);
// 	float s_y = surface(1);
// 	float s_z = surface(2);

// 	Vector3f sat_xyz = Vector3f(ControlMath::sat(s_x/0.5f),ControlMath::sat(s_y/0.5f),ControlMath::sat(s_z/0.5f));

// 	Vector3f acc_sp_velocity = vel_error.emult(lam_ps) + (sat_xyz.emult(k_1));
// 	// No control input from setpoints or corresponding states which are NAN
// 	ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);
// 	_accelerationControl();

// 	// Saturate maximal vertical thrust
// 	_thr_sp(2) = math::max(_thr_sp(2), -_lim_thr_max);

// 	// Get allowed horizontal thrust after prioritizing vertical control
// 	const float thrust_max_squared = _lim_thr_max * _lim_thr_max;
// 	const float thrust_z_squared = _thr_sp(2) * _thr_sp(2);
// 	const float thrust_max_xy_squared = thrust_max_squared - thrust_z_squared;
// 	float thrust_max_xy = 0;

// 	if (thrust_max_xy_squared > 0) {
// 		thrust_max_xy = sqrtf(thrust_max_xy_squared);
// 	}

// 	// Saturate thrust in horizontal direction
// 	const Vector2f thrust_sp_xy(_thr_sp);
// 	const float thrust_sp_xy_norm = thrust_sp_xy.norm();

// 	if (thrust_sp_xy_norm > thrust_max_xy) {
// 		_thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
// 	}


// 	// u_x = _acc_sp(0) + lam_ps(0)*vel_errorr(0) + (k_1(0)*ControlMath::sat(surface(0)/0.5));
// 	// u_y = _acc_sp(1) + lam_ps(1)*vel_errorr(1) + (k_1(1)*ControlMath::sat(surface(1)/0.5));

// 	// float K_phi = 1;
// 	// float K_theta = 1;
// 	// float k_1_x = 1 ;
// 	// float k_1_y = 1;

// 	// theta = CONSTANTS_MASS/_thr_sp(2) * (cos(_yaw_sp)*(-_acc_sp(0) + K_theta*ex_dot) + sin(_yaw_sp)*(-_acc_sp(1) + K_phi * ey_dot)) + cos(_yaw_sp)/_thr_sp(2)*k_1_x*ControlMath::sat(s_x/1) + sin(_yaw_sp)/_thr_sp(2)*k_1_y*ControlMath::sat(s_y/1);
//         // phi   = CONSTANTS_MASS/_thr_sp(2) * (sin(_yaw_sp)*(-_acc_sp(0) + K_theta*ex_dot) - cos(_yaw_sp)*(-_acc_sp(1) + K_phi * ey_dot)) + sin(_yaw_sp)/_thr_sp(2)*k_1_x*ControlMath::sat(s_x/1) - cos(_yaw_sp)/_thr_sp(2)*k_1_y*ControlMath::sat(s_y/1);

// 	// float u_z = _acc_sp(2) + lam_ps(2)*vel_errorr(2) + (k_1(2)*ControlMath::sat(surface(2)/0.5));

// 	// float u_xyz = _acc_sp + lam_ps*vel_errorr + (k_1*ControlMath::sat(surface(0)/0.5));

// 	// float u1 = pow(u_x,2) + pow(u_y,2) + pow(u_z + CONSTANTS_MASS , 2) ;
// 	// Vector3f u1_squrt = Vector3f(0, 0, sqrtf(u1)).normalized();

// }

void PositionControl::_positionControl()
{
	// P-position controller
	Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);
	// Position and feed-forward velocity setpoints or position states being NAN results in them not having an influence
	ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
	// make sure there are no NAN elements for further reference while constraining
	ControlMath::setZeroIfNanVector3f(vel_sp_position);

	// Constrain horizontal velocity by prioritizing the velocity component along the
	// the desired position setpoint over the feed-forward term.
	_vel_sp.xy() = ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal);
	// Constrain velocity in z-direction.
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);
}

void PositionControl::_velocityControl(const float dt)
{

	// SMC control
	Vector3f vel_error = _vel_sp - _vel;
	// Gain for ST term
	Vector3f K_s_1 = Vector3f(0.75,0.75,0.8); /// 0.7 , 0.7 , 1
	Vector3f K_s_2 = Vector3f(0.9,0.9,1);/// 1 1 1
	// Vector3f surface = vel_error + (_pos_sp - _pos).emult(_gain_pos_p);
	Vector3f surface = vel_error ;
	float s_x = surface(0);
	float s_y = surface(1);
	float s_z = surface(2);
	float size_sat = 0.5f;
	_vel_int = Vector3f(ControlMath::sat(s_x/size_sat),ControlMath::sat(s_y/size_sat),ControlMath::sat(s_z/size_sat));
	Vector3f sat_xyz = _vel_int;
	// // Update integral part of v_xyz (Switching term)
        _vel_int += _vel_int.emult(K_s_2)*dt;
	//limit thrust integral
	_vel_int(2) = math::min(fabsf(_vel_int(2)), CONSTANTS_ONE_G) * sign(_vel_int(2));
	// Total  SUPER twsing term
	Vector3f SW = Vector3f (_vel_int(0) + sqrt(fabsf(sat_xyz(0))*K_s_1(0)),_vel_int(1) + sqrt(fabsf(sat_xyz(1)))*(K_s_1(1)),_vel_int(2) + sqrt(fabsf(sat_xyz(2)))*(K_s_1(2)));


	// Vector3f Switch_cd = Vector3f(ControlMath::sat(s_x/0.5f),ControlMath::sat(s_y/0.5f),ControlMath::sat(s_z/0.5f));
	// Virtual acceleration
	Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) + SW;
	// Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d);

	// Vector3f surface = vel_error + (_pos_sp - _pos).emult(_gain_pos_p);
	// Vector3f acc_sp_velocity = vel_error.emult(lam_ps) + (k_1(0)*ControlMath::sat(surface(0)/0.5));

	// No control input from setpoints or corresponding states which are NAN
	ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

	_accelerationControl();

	// Integrator anti-windup in vertical direction
	if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.0f) ||
	    (_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.0f)) {
		vel_error(2) = 0.f;
	}

	// Saturate maximal vertical thrust
	_thr_sp(2) = math::max(_thr_sp(2), -_lim_thr_max);

	// Get allowed horizontal thrust after prioritizing vertical control
	const float thrust_max_squared = _lim_thr_max * _lim_thr_max;
	const float thrust_z_squared = _thr_sp(2) * _thr_sp(2);
	const float thrust_max_xy_squared = thrust_max_squared - thrust_z_squared;
	float thrust_max_xy = 0;

	if (thrust_max_xy_squared > 0) {
		thrust_max_xy = sqrtf(thrust_max_xy_squared);
	}

	// Saturate thrust in horizontal direction
	const Vector2f thrust_sp_xy(_thr_sp);
	const float thrust_sp_xy_norm = thrust_sp_xy.norm();

	if (thrust_sp_xy_norm > thrust_max_xy) {
		_thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
	}

	// Use tracking Anti-Windup for horizontal direction: during saturation, the integrator is used to unsaturate the output
	// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
	const Vector2f acc_sp_xy_limited = Vector2f(_thr_sp) * (CONSTANTS_ONE_G / _hover_thrust);
	const float arw_gain = 2.f / _gain_vel_p(0);
	vel_error.xy() = Vector2f(vel_error) - (arw_gain * (Vector2f(_acc_sp) - acc_sp_xy_limited));

	// Make sure integral doesn't get NAN
	ControlMath::setZeroIfNanVector3f(vel_error);
	// // Update integral part of velocity control
	// _vel_int += vel_error.emult(_gain_vel_i) * dt;

	// // limit thrust integral
	// _vel_int(2) = math::min(fabsf(_vel_int(2)), CONSTANTS_ONE_G) * sign(_vel_int(2));
}

void PositionControl::_accelerationControl()
{
	// Assume standard acceleration due to gravity in vertical direction for attitude generation
	Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), CONSTANTS_ONE_G).normalized();
	ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);
	// Scale thrust assuming hover thrust produces standard gravity
	float collective_thrust = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
	// Project thrust to planned body attitude
	collective_thrust /= (Vector3f(0, 0, 1).dot(body_z));
	collective_thrust = math::min(collective_thrust, -_lim_thr_min);
	_thr_sp = body_z * collective_thrust;
}

bool PositionControl::_updateSuccessful()
{
	bool valid = true;

	// For each controlled state the estimate has to be valid
	for (int i = 0; i <= 2; i++) {
		if (PX4_ISFINITE(_pos_sp(i))) {
			valid = valid && PX4_ISFINITE(_pos(i));
		}

		if (PX4_ISFINITE(_vel_sp(i))) {
			valid = valid && PX4_ISFINITE(_vel(i)) && PX4_ISFINITE(_vel_dot(i));
		}
	}

	// There has to be a valid output accleration and thrust setpoint otherwise there was no
	// setpoint-state pair for each axis that can get controlled
	valid = valid && PX4_ISFINITE(_acc_sp(0)) && PX4_ISFINITE(_acc_sp(1)) && PX4_ISFINITE(_acc_sp(2));
	valid = valid && PX4_ISFINITE(_thr_sp(0)) && PX4_ISFINITE(_thr_sp(1)) && PX4_ISFINITE(_thr_sp(2));
	return valid;
}

void PositionControl::getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const
{
	local_position_setpoint.x = _pos_sp(0);
	local_position_setpoint.y = _pos_sp(1);
	local_position_setpoint.z = _pos_sp(2);
	local_position_setpoint.yaw = _yaw_sp;
	local_position_setpoint.yawspeed = _yawspeed_sp;
	local_position_setpoint.vx = _vel_sp(0);
	local_position_setpoint.vy = _vel_sp(1);
	local_position_setpoint.vz = _vel_sp(2);
	_acc_sp.copyTo(local_position_setpoint.acceleration);
	_thr_sp.copyTo(local_position_setpoint.thrust);
}

void PositionControl::getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const
{

	// attitude_setpoint.pitch_body = phi;
	// attitude_setpoint.roll_body = theta;
	// attitude_setpoint.yaw_body = _yaw_sp;
	ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, attitude_setpoint);
	attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;
}
