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

#define BACK_STEPPING_CONTROLLER false
#define SMC_CONTROLLER false
#define SWSMC  false
#define OBSERVER true
#define OBSERVER_2 false

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

	// _positionControl();
	// _velocityControl(dt);
	SMC_control(dt);
	_yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
	_yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw; // TODO: better way to disable yaw control

	return valid && _updateSuccessful();
}


// void PositionControl::SMC_control(const float dt)
// {

// 	// // PD controle
// 	// Vector3f acc_sp_velocity = _acc_sp + vel_error.emult(_gain_vel_p) + (_pos_sp - _pos).emult(_gain_pos_p);



void PositionControl::SMC_control(const float dt)
{
	//--- Back-stepping controller ---

	#if BACK_STEPPING_CONTROLLER

		//constante de perturbation K1, K2
		Vector3f  k1 = Vector3f(4*0.35, 4*0.45, 1.5*1.3);
		Vector3f  k2 = Vector3f(4*0.35, 4*0.45, 1.5*2);
		//delta pos
		Vector3f delta_pos = _pos_sp-_pos;
		//delta speed
		Vector3f delta_speed = _vel - _vel_sp - delta_pos.emult(k1);
		// Vector3f pert = Vector3f(X_chap_[6], X_chap_[7], X_chap_[8]);
 		Vector3f acc_sp_velocity = (delta_speed+delta_pos.emult(k1)).emult(-k1)-delta_speed.emult(k2);

		// No control input from setpoints or corresponding states which are NAN
		ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

		if (!isnan(_pos_sp(0)) && !isnan(_pos_sp(1)))
    		{
		FILE *fichier = fopen("Back_Stepping_info.txt","a");

			double timeA = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();

		fprintf(fichier,"pos_setpoint :","\t%f\t%f\t%f\n" , "pos_current:","\t%f\t%f\t%f\n","time:", "\t%f\n",(double)_pos_sp(0), (double)_pos_sp(1), (double)_pos_sp(2),
									(double)_pos(0), (double)_pos(1), (double)_pos(2),
									(double) (timeA-timed));

		fclose(fichier);
		}

	#endif
			//--- fin black stepping controler ---



			//--- SMC Controller ---

	#if SMC_CONTROLLER

		//constante de perturbation K1, K2
		// Vector3f pert = Vector3f(X_chap_[6], X_chap_[7], X_chap_[8]);
		Vector3f  k1 = Vector3f(4*0.35, 4*0.45, 1.5*1.3);
		Vector3f  k2 = Vector3f(4*0.35, 4*0.45, 1.5*2);
		Vector3f e = _pos - _pos_sp;
		Vector3f e_dot = _vel - _vel_sp;
		Vector3f S = k1.emult(e) + e_dot;
		// Saturate S
		// for (int i=0; i<3; i++) {
		//     if (!(-1 < S(i) && S(i) < 1 ))
		//     	S(i) = (S(i)>=0)?1:-1;
		// }
		// float size_sat = 1;
		// Vector3f sat_xyz = Vector3f(ControlMath::sat(S(0)/size_sat),ControlMath::sat(S(1)/size_sat),ControlMath::sat(S(2)/size_sat));
		// Vector3f SW = Vector3f ((sat_xyz(0))*k2(0),(sat_xyz(1))*k2(1), (sat_xyz(2))*k2(2));
		Vector3f SW = Vector3f (sign(S(0))*k2(0),sign(S(1))*k2(1), sign(S(2))*k2(2));
		Vector3f acc_sp_velocity = e_dot.emult(k1) + SW ;

		ControlMath::addIfNotNanVector3f(_acc_sp, -acc_sp_velocity);

		if (!isnan(_pos_sp(0)) && !isnan(_pos_sp(1)))
    		{
		FILE *fichier = fopen("SMC_info.txt","a");

		double timeA = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();

		fprintf(fichier,"pos_setpoint : \t%f\t%f\t%f\n , pos_current: \t%f\t%f\t%f\n time: \t%f\n",(double)_pos_sp(0), (double)_pos_sp(1), (double)_pos_sp(2),
									(double)_pos(0), (double)_pos(1), (double)_pos(2),
									(double) (timeA-timed));

		fclose(fichier);
		}

	#endif
			//--- Fin SMC Controller ---



	#if  OBSERVER

		// float 	ax1 = 1.0, ay1 = 1.0, az1 = 1.0,
		//  	ax2 = 3.2, ay2 = 3.2, az2 = 1.55,
		//  	ax3 = 3.5, ay3 = 3.5, az3 = 1.8;

		// Gain of Controller
		float 	k1x = 2,k1y = 2,k1z = 1.55;
		float	k2x = 2,k2y = 2,k2z = 1;
		float	c1x = 1.5, c1y=1.5, c1z = 1;



		// Gain of Observer
		float 	lamda1x = 6,lamda2x = 20,lamda3x = 5;
		float	lamda1y = 6,lamda2y = 20,lamda3y = 5;



		// compute error
		// sigma_0 ... estimation error
		ControlMath::setZeroIfNanVector3f(xp_x);
		ControlMath::setZeroIfNanVector3f(xp_y);


		// OBSERVER based control
		// init value for observator
		// Vector3f Phi1 = Vector3f(1.0f , dt , dt*dt/2);
		// Vector3f Phi2 = Vector3f(0.0f ,  1.0f ,   dt);
		// Vector3f Phi3 = Vector3f(0.0f , 0.0f , 1.0f);
		// float c = 3;

		if (!PX4_ISFINITE(Uxx) || !PX4_ISFINITE(Uyy) ){
			Uxx = 0.0f;
			Uyy = 0.0f;
		}

		float U_comp_x = Uxx ;
		float U_comp_y = Uyy ;

		// float z_x = 0;
		// float z_y = 0;

		float sigma_0_x = _pos(0) - xp_x(0) ;
		float sigma_0_y = _pos(1) - xp_y(0) ;

		// if (abs(sigma_0_x) > 0) {
		// 	// current eigenvalue
		// 	// float s_x = -c *( abs( sigma_0_x ) ) ^(-1/3) ;
		// 	float s_x = -c/cbrt(abs( sigma_0_x ));
		// 	// Matching discretization approach
		// 	z_x = exp( dt * s_x );
		// 	// z = 1/(1 -(s* dt )); % Implicit discretization approach
		// }
		// if (abs(sigma_0_y) > 0) {
		// 	// current eigenvalue
		// 	float s_y = -c/cbrt(abs( sigma_0_y ));
		// 	// Matching discretization approach
		// 	z_y = exp(dt * s_y);
		// 	// z = 1/(1 -(s* dt )); % Implicit discretization approach
		// }

		// // % Second Order Differentiator
		// float lamda1_x = (3 - 3*z_x)*abs(sigma_0_x) ;
		// // float lamda2_x = ((( z_x - 1) / dt * abs( sigma_0_x ) ^(1/2) ) ^2*( z_x + 5) ) * dt /2;
		// // float lamda3_x = -(( z_x - 1) / dt * abs( sigma_0_x ) ^(1/3) ) ^3*dt
		// float lamda2_x = (z_x - 1)/dt * sqrt(abs( sigma_0_x ))*(z_x - 1)/dt * sqrt(abs( sigma_0_x ))*(z_x + 5)*dt/2;
		// float a_x = cbrt(abs( sigma_0_x ));
		// float lamda3_x = -((z_x - 1)/dt * a_x)*((z_x - 1)/dt * a_x)*((z_x - 1)/dt * a_x)*dt;

		// float lamda1_y = (3 - 3*z_y)*abs( sigma_0_y );
		// float lamda2_y = ( z_y - 1)/dt * sqrt(abs( sigma_0_y ))*( z_y - 1)/dt * sqrt(abs( sigma_0_y ))*(z_y + 5)*dt /2;
		// float a_y = cbrt(abs( sigma_0_y ));
		// float lamda3_y = -(( z_y - 1) / dt * a_y)*(( z_y - 1) / dt * a_y)*(( z_y - 1) / dt * a_y)*dt;

		// Vector3f lambda_x = Vector3f(lamda1_x , lamda2_x , lamda3_x);
		// Vector3f lambda_y = Vector3f(lamda1_y , lamda2_y , lamda3_y);

		// // ////% Compute Estimates
		// xp_x = Vector3f(xp_x.dot(Phi1),xp_x.dot(Phi2),xp_x.dot(Phi3)) + lambda_x * sign ( sigma_0_x ) ;
		// xp_x(1) = xp_x(1) + dt*U_comp_x;
		// xp_y = Vector3f(xp_y.dot(Phi1),xp_y.dot(Phi2),xp_y.dot(Phi3)) + lambda_y * sign ( sigma_0_y ) ;
		// xp_y(1) = xp_y(1) + dt*U_comp_y;

		float sat_sigmax = ControlMath::sat(sigma_0_x/0.5f);
		float sat_sigmay = ControlMath::sat(sigma_0_y/0.5f);

		xp_x(0) = xp_x(0) + dt*lamda1x *cbrtf(fabsf(sigma_0_x)*fabsf(sigma_0_x))*sign(sigma_0_x) + (dt*xp_x(1) + (dt*dt/2)*xp_x(2)) ;
    		xp_x(1) = xp_x(1) + dt*lamda2x*cbrtf(fabsf(sigma_0_x))*sign(sigma_0_x) + (dt*xp_x(2)) +  dt*(U_comp_x);
    		xp_x(2) = xp_x(2) + dt*lamda3x*sat_sigmax ;
		// xp_x(2) = xp_x(2) + dt*lamda3x*sign(sigma_0_x) ;

		xp_y(0) = xp_y(0) + dt*lamda1y *cbrtf(fabsf(sigma_0_y)*fabsf(sigma_0_y)*1.0f)*sign(sigma_0_y) + (dt*xp_y(1) + (dt*dt/2)*xp_y(2)) ;
    		xp_y(1) = xp_y(1) + dt*lamda2y*cbrtf(fabsf(sigma_0_y))*sign(sigma_0_y) + (dt*xp_y(2)) +  dt*(U_comp_y);
		xp_y(2) = xp_y(2) + dt*lamda3y*sat_sigmay ;
    		// xp_y(2) = xp_y(2) + dt*lamda3y*sign(sigma_0_y) ;

		// // low pass filtre for estimated disturbance
		// float dist_x = ControlMath::Lowpass(xp_x(2) ,0.5f, dt ,Statex);
		// float dist_y = ControlMath::Lowpass(xp_y(2) ,0.5f, dt ,Statey);

		//// xp_x(0) = x_hat , xp_x(1) = x_hat_dot , xp_x(2) = disturbance_x
		//// xp_y(0) = y_hat , xp_y(1) = y_hat_dot , xp_y(2) = disturbance_y

		float error_x =_pos(0) -_pos_sp(0);
		float error_y =_pos(1) -_pos_sp(1);
		float error_z =_pos(2) -_pos_sp(2);

		float sx= c1x*error_x +(xp_x(1) -_vel_sp(0)) ;
		float sy= c1y*error_y +(xp_y(1) -_vel_sp(1)) ;
		// z use only Super-Twisting SMC
		float sz= c1z*error_z +(_vel(2) -_vel_sp(2)) ;

		float sat_sx = ControlMath::sat(sx);
		float sat_sy = ControlMath::sat(sy);
		float sat_sz = ControlMath::sat(sz);

		float sat_ex = ControlMath::sat((_pos(0) - xp_x(0)));
		float sat_ey = ControlMath::sat((_pos(1) - xp_y(0)));
		// float sat_ez = ControlMath::sat(_pos(2) -_pos_sp(2));

		sat_sx_int += k2x*sat_sx*dt;
		sat_sy_int += k2y*sat_sy*dt;
		sat_sz_int += k2z*sat_sz*dt;

		// sat_ex_int += sat_ex*dt;
		// sat_ey_int += sat_ey*dt;
		// sat_ez_int += sat_ez*dt;

		float Ux = 0;
		float Uy = 0;
		float Uz = 0;


		// Ux =  c1x*(-xp_x(1) +_vel_sp(0)) - xp_x(2) - lamda2x*cbrt(abs(xp_x(0) -_pos_sp(0)))*sat_ex -k1x*sqrt(abs(sx))*sat_sx - sat_sx_int ;
		// Uy =  c1y*(-xp_y(1) +_vel_sp(1)) - xp_y(2) - lamda2y*cbrt(abs(xp_y(0) -_pos_sp(1)))*sat_ey -k1y*sqrt(abs(sy))*sat_sy - sat_sy_int;
		Ux =  c1x*(-xp_x(1) +_vel_sp(0)) - xp_x(2) - lamda2x*cbrtf(fabsf(xp_x(0) -_pos_sp(0)))*sat_ex -k1x*sqrtf(fabsf(sx))*sat_sx - sat_sx_int ;
		Uy =  c1y*(-xp_y(1) +_vel_sp(1)) - xp_y(2) - lamda2y*cbrtf(fabsf(xp_y(0) -_pos_sp(1)))*sat_ey -k1y*sqrtf(fabsf(sy))*sat_sy - sat_sy_int;
		Uz = c1z*(-_vel(2) +_vel_sp(2)) - k1z*sqrtf(fabsf(sz))*sat_sz - sat_sz_int;

		Uxx = Ux;
		Uyy = Uy;

			// All nan if not takeoff or have a mission
		Vector3f U = Vector3f(Ux, Uy, Uz);
		ControlMath::addIfNotNanVector3f(_acc_sp, U);

		FILE *fichier = fopen("Obser.txt","a");
		fprintf(fichier,"  %f\t  %f\t  %f\t   %f\t  %f\t   %f\t    %f\t   %f\t 	%f\t  %f\t   %f\t    %f\t   %f\t   %f\t  %f\t   %f\t    %f\t   %f\t   %f\n",(double)xp_x(1) , (double)_vel(0), (double)xp_y(1),
									(double)_vel(1) , (double)_vel(2) ,(double)xp_x(2),(double)xp_y(2),(double)Ux,
									(double)Uy,(double)Uz,(double)_pos_sp(0), (double)_pos_sp(1), (double)_pos_sp(2),
									(double)_pos(0), (double)_pos(1), (double)_pos(2),
									(double)_vel_sp(0), (double)_vel_sp(1), (double)_vel_sp(2));

		fclose(fichier);


	#endif

		#if  OBSERVER_2

		// float 	ax1 = 1.0, ay1 = 1.0, az1 = 1.0,
		//  	ax2 = 3.2, ay2 = 3.2, az2 = 1.55,
		//  	ax3 = 3.5, ay3 = 3.5, az3 = 1.8;

		// Gain of Controller
		float 	k1x = 2,k1y = 2,k1z = 2;
		float	k2x = 2,k2y = 2,k2z = 2;
		float	k3x = 2,k3y = 2,k3z = 2;
		float	k4x = 2,k4y = 2,k4z = 2;
		float	c1x = 1, c1y=1, c1z = 1;

		// Gain of Observer
		float 	lamda1x = 6,lamda2x = 20,lamda3x = 5;
		float	lamda1y = 6,lamda2y = 20,lamda3y = 5;

		// compute error
		// sigma_0 ... estimation error
		ControlMath::setZeroIfNanVector3f(xp_x);
		ControlMath::setZeroIfNanVector3f(xp_y);


		// OBSERVER based control
		// init value for observator
		// Vector3f Phi1 = Vector3f(1.0f , dt , dt*dt/2);
		// Vector3f Phi2 = Vector3f(0.0f ,  1.0f ,   dt);
		// Vector3f Phi3 = Vector3f(0.0f , 0.0f , 1.0f);
		// float c = 5;

		if (!PX4_ISFINITE(Uxx) || !PX4_ISFINITE(Uyy) ){
			Uxx = 0.0f;
			Uyy = 0.0f;
		}

		float U_comp_x = Uxx ;
		float U_comp_y = Uyy ;

		// float z_x = 0;
		// float z_y = 0;

		float sigma_0_x = _vel(0) - xp_x(0) ;
		float sigma_0_y = _vel(1) - xp_y(0) ;

		// if (abs(sigma_0_x) > 0) {
		// 	// current eigenvalue
		// 	// float s_x = -c *( abs( sigma_0_x ) ) ^(-1/3) ;
		// 	float s_x = -c/cbrt(fabsf( sigma_0_x ));
		// 	// Matching discretization approach
		// 	z_x = exp( dt * s_x );
		// 	// z = 1/(1 -(s* dt )); % Implicit discretization approach
		// }
		// if (abs(sigma_0_y) > 0) {
		// 	// current eigenvalue
		// 	float s_y = -c/cbrt(fabsf( sigma_0_y ));
		// 	// Matching discretization approach
		// 	z_y = exp(dt * s_y);
		// 	// z = 1/(1 -(s* dt )); % Implicit discretization approach
		// }

		// // % Second Order Differentiator
		// float lamda1_x = (3 - 3*z_x)*fabsf(sigma_0_x) ;
		// float lamda2_x = (z_x - 1)/dt * sqrt(fabsf( sigma_0_x ))*(z_x - 1)/dt * sqrt(fabsf( sigma_0_x ))*(z_x + 5)*dt/2;
		// float a_x = cbrt(fabsf( sigma_0_x ));
		// float lamda3_x = -((z_x - 1)/dt * a_x)*((z_x - 1)/dt * a_x)*((z_x - 1)/dt * a_x)*dt;

		// float lamda1_y = (3 - 3*z_y)*fabsf( sigma_0_y );
		// float lamda2_y = ( z_y - 1)/dt * sqrt(fabsf( sigma_0_y ))*( z_y - 1)/dt * sqrt(fabsf( sigma_0_y ))*(z_y + 5)*dt /2;
		// float a_y = cbrt(fabsf( sigma_0_y ));
		// float lamda3_y = -(( z_y - 1) / dt * a_y)*(( z_y - 1) / dt * a_y)*(( z_y - 1) / dt * a_y)*dt;

		// Vector3f lambda_x = Vector3f(lamda1_x , lamda2_x , lamda3_x);
		// Vector3f lambda_y = Vector3f(lamda1_y , lamda2_y , lamda3_y);

		// ////% Compute Estimates
		// xp_x = Vector3f(xp_x.dot(Phi1),xp_x.dot(Phi2),xp_x.dot(Phi3)) + lambda_x * sign ( sigma_0_x ) ;
		// xp_x(0) = xp_x(0) - dt*U_comp_x;
		// xp_y = Vector3f(xp_y.dot(Phi1),xp_y.dot(Phi2),xp_y.dot(Phi3)) + lambda_y * sign ( sigma_0_y ) ;
		// xp_y(0) = xp_y(0) - dt*U_comp_y;
		float sat_sigmax = ControlMath::sat(sigma_0_x);
		float sat_sigmay = ControlMath::sat(sigma_0_y);

		xp_x(0) = xp_x(0) + dt*lamda1x *cbrt(fabsf(sigma_0_x)*fabsf(sigma_0_x))*sign(sigma_0_x) + (dt*xp_x(1) + (dt*dt/2)*xp_x(2)) +  dt*(U_comp_x);
    		xp_x(1) = xp_x(1) + dt*lamda2x*cbrt(fabsf(sigma_0_x))*sat_sigmax + (dt*xp_x(2)) ;
    		xp_x(2) = xp_x(2) + dt*lamda3x*sat_sigmax ;


		xp_y(0) = xp_y(0) + dt*lamda1y *cbrt(fabsf(sigma_0_y)*fabsf(sigma_0_y))*sign(sigma_0_y) + (dt*xp_y(1) + (dt*dt/2)*xp_y(2)) +  dt*(U_comp_y);
    		xp_y(1) = xp_y(1) + dt*lamda2y*cbrt(fabsf(sigma_0_y))*sat_sigmay + (dt*xp_y(2)) ;
    		xp_y(2) = xp_y(2) + dt*lamda3y*sat_sigmay ;

		// low pass filtre for estimated disturbance
		// float dist_x = ControlMath::Lowpass(xp_x(1) ,1.0f, dt ,Statex);
		// float dist_y = ControlMath::Lowpass(xp_y(1) ,1.0f, dt ,Statey);

		//// xp_x(0) = x_hat , xp_x(1) = x_hat_dot , xp_x(2) = disturbance_x
		//// xp_y(0) = y_hat , xp_y(1) = y_hat_dot , xp_y(2) = disturbance_y

		float error_x =_pos(0) -_pos_sp(0);
		float error_y =_pos(1) -_pos_sp(1);
		float error_z =_pos(2) -_pos_sp(2);

		float sx= c1x*error_x +(_vel(0) -_vel_sp(0)) ;
		float sy= c1y*error_y +(_vel(1) -_vel_sp(1)) ;
		float sz= c1z*error_z +(_vel(2) -_vel_sp(2)) ;
		// satured instead of sign function
		float sat_sx = ControlMath::sat(sx);
		float sat_sy = ControlMath::sat(sy);
		float sat_sz = ControlMath::sat(sz);

		float sat_ex = ControlMath::sat((_vel(0) - xp_x(0)));
		float sat_ey = ControlMath::sat((_vel(1) - xp_y(0)));

		// Make sure integral doesn't get NAN
		if (!PX4_ISFINITE(sat_sx) || !PX4_ISFINITE(sat_sy) || !PX4_ISFINITE(sat_sz) ){
			sat_sx = 0.0f;
			sat_sy = 0.0f;
			sat_sz = 0.0f;
		}
		// take intergale
		sat_sx_int += k2x*sat_sx*dt;
		sat_sy_int += k2y*sat_sy*dt;
		sat_sz_int += k2z*sat_sz*dt;

		// addition for modified STSMC
		SW3_x +=  k4x*sx*dt;
		SW3_y +=  k4y*sy*dt;
		SW3_z +=  k4z*sz*dt;

		float Ux = 0;
		float Uy = 0;
		float Uz = 0;
		// Modified STSMC control
		Ux = -xp_x(1) + c1x*(-_vel(0) +_vel_sp(0))- k1x*sqrt(fabsf(sx))*sat_sx - sat_sx_int - k3x*sat_sx - SW3_x -
							lamda1x *cbrt(fabsf(sigma_0_x)*fabsf(sigma_0_x))*sign(_vel(0) - xp_x(0)) ;
		Uy = -xp_y(1) + c1y*(-_vel(1) +_vel_sp(1)) -k1y*sqrt(fabsf(sy))*sat_sy - sat_sy_int - k3y*sat_sy - SW3_y -
							lamda1y *cbrt(fabsf(sigma_0_y)*fabsf(sigma_0_y))*sign(_vel(1) - xp_y(0));
		Uz = c1z*(-_vel(2) +_vel_sp(2)) - k1z*sqrt(fabsf(sz))*sat_sz - sat_sz_int - k3z*sat_sz - SW3_z;

		Uxx = Ux;
		Uyy = Uy;

			// All nan if not takeoff or have a mission
		Vector3f U = Vector3f(Ux, Uy, Uz);
		ControlMath::addIfNotNanVector3f(_acc_sp, U);

		FILE *fichier = fopen("Obser2.txt","a");
		fprintf(fichier,"  %f\t  %f\t  %f\t   %f\t  %f\t   %f\t    %f\t   %f\t   %f\n",(double)xp_x(0) , (double)_vel(0), (double)xp_y(0),
									(double)_vel(1) , (double)xp_x(1),
									(double)xp_y(1),(double)Ux,
									(double)Uy,(double)sat_sx_int);

		fclose(fichier);
	#endif

			//--- Super twisting sliding mode controller (SWSMC)  ---
	#if SWSMC

		float 	ax1 = 1.0, ay1 = 1.0, az1 = 1.0,
		 	ax2 = 3.2, ay2 = 3.2, az2 = 1.55,
		 	ax3 = 3.0, ay3 = 3.0, az3 = 1.0;
		float Ux = 0.0, Uy = 0.0, Uz = 0.0;

		ControlMath::setZeroIfNanVector3f(xp_x);

		Vector3f ep = _pos_sp - _pos;
		Vector3f epdot = _vel_sp - _vel;

		float sx= epdot(0) + ax1*ep(0);
		float sy= epdot(1) + ay1*ep(1);
		float sz= epdot(2) + az1*ep(2);

		float sat_sx = ControlMath::sat(sx);
		float sat_sy = ControlMath::sat(sy);
		float sat_sz = ControlMath::sat(sz);

		sat_sxin += sat_sx*dt;
		sat_syin += sat_sy*dt;
		sat_szin += sat_sz*dt;


		Ux = ax1*epdot(0)+ax2*sqrt(fabsf(sx))*sat_sx +ax3*sat_sxin ;
		Uy = ay1*epdot(1)+ay2*sqrt(fabsf(sy))*sat_sy +ay3*sat_syin ;
		Uz = az1*epdot(2)+az2*sqrt(fabsf(sz))*sat_sz +az3*sat_szin ;


		Vector3f U = Vector3f(Ux, Uy, Uz);
		ControlMath::addIfNotNanVector3f(_acc_sp, U);
		// FILE *fichier = fopen("SWSMC.txt","a");
		// fprintf(fichier,"  %f\t  %f\t  %f\t   %f\t  %f\t  %f  \t%f\t  %f\t  %f\n",(double)xp_x(0) , (double)xp_x(1), (double)xp_x(2),
		// 							(double)sat_sxin , (double)sat_syin ,(double)sat_szin,
		// 							(double)Ux , (double)Uy ,(double)Uz);

		// fclose(fichier);
		FILE *fichier = fopen("SWSMC.txt","a");
		fprintf(fichier,"  %f\t  %f\t  %f\t  \n",(double)Ux , (double)Uy ,(double)Uz);
		fclose(fichier);

	#endif
			// fin 3ème Controller ---

	_accelerationControl();

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


}

//// Original code PID of PX4
// void PositionControl::_positionControl()
// {
// 	// P-position controller
// 	Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);
// 	// Position and feed-forward velocity setpoints or position states being NAN results in them not having an influence
// 	ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
// 	// make sure there are no NAN elements for further reference while constraining
// 	ControlMath::setZeroIfNanVector3f(vel_sp_position);

// 	// // Constrain horizontal velocity by prioritizing the velocity component along the
// 	// // the desired position setpoint over the feed-forward term.
// 	_vel_sp.xy() = ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal);
// 	// Constrain velocity in z-direction.
// 	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);
// }

// void PositionControl::_velocityControl(const float dt)
// {
// 	// PID velocity control
// 	Vector3f vel_error = _vel_sp - _vel;
// 	Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d);


// 	// No control input from setpoints or corresponding states which are NAN
// 	 ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);


// 	_accelerationControl();

// 	// Integrator anti-windup in vertical direction
// 	if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.0f) ||
// 	    (_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.0f)) {
// 		vel_error(2) = 0.f;
// 	}

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

// 	// Use tracking Anti-Windup for horizontal direction: during saturation, the integrator is used to unsaturate the output
// 	// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
// 	const Vector2f acc_sp_xy_limited = Vector2f(_thr_sp) * (CONSTANTS_ONE_G / _hover_thrust);
// 	const float arw_gain = 2.f / _gain_vel_p(0);
// 	vel_error.xy() = Vector2f(vel_error) - (arw_gain * (Vector2f(_acc_sp) - acc_sp_xy_limited));

// 	// Make sure integral doesn't get NAN
//	ControlMath::setZeroIfNanVector3f(vel_error);
//	// Update integral part of velocity control
// 	_vel_int += vel_error.emult(_gain_vel_i) * dt;
// 	FILE *fichier = fopen("SWSMC.txt","a");
// 	fprintf(fichier,"  %f\t  %f\t  %f\t   %f  \n",(double)_vel_int(0) , (double)_vel_int(1), (double)_vel_int(2),(double)dt);
// 	fclose(fichier);

// 	// limit thrust integral
// 	_vel_int(2) = math::min(fabsf(_vel_int(2)), CONSTANTS_ONE_G) * sign(_vel_int(2));
// }

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
	ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, attitude_setpoint);
	attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;
}
