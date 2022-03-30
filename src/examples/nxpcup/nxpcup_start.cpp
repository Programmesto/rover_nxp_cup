/****************************************************************************
 *
 *   Copyright 2019 NXP.
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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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
 * @file nxpcup_main.cpp
 * Race code for NXP Cup
 *
 * @author Katrin Moritz
 */

#include "nxpcup_start.h"
#include "Pixy2I2C_PX4.h"
#include "nxpcup_race.h"

#include <math.h>
#include <px4_defines.h>
#include <stdlib.h>

#define PI 3.14159265

double lengthVector(Vector* vector);
double calculateAngle(Vector* vector);
roverControl raceTrack(Pixy2 &pixy);
void sortVector(Vector* vector);

static int daemon_task;             /* Handle of deamon task / thread */

bool threadShouldExit = false;
bool threadIsRunning = false;
int counter = 0;

//скорость при повороте
float turn_speed = 0.2;
//скорость на прямой
float straight_speed = 0.25;

//скорость на которой постоянно ездит машина
float speed = 0.0;

//последний угол поворота
float steer = 0.0;
double angle_for_turn = 45.0;
double correction_angle = 20;
double kf_angle = 1.3;
int max_counter = 1;
bool race = true;

void roverSteerSpeed(roverControl control, vehicle_attitude_setpoint_s &_att_sp)
{
	// Converting steering value from percent to euler angle
	control.steer *= 0.7f;//60.0f; //max turn angle 60 degree

	// Converting steering value from percent to euler angle
	control.steer *= -60.0f; //max turn angle 60 degree
	control.steer *= (float)3.14159 / 180; // change to radians

	// Converting the euler angle into a quaternion for vehicle_attitude_setpoint
	matrix::Eulerf euler{0.0, 0.0, control.steer};
	matrix::Quatf qe{euler};

	// Throttle control of the rover
	_att_sp.thrust_body[0] = control.speed;

	// Steering control of the Rover
	_att_sp.q_d[0] = qe(0);
	_att_sp.q_d[1] = qe(1);
	_att_sp.q_d[2] = qe(2);
	_att_sp.q_d[3] = qe(3);

}

int race_thread_main(int argc, char **argv)
{
	threadIsRunning = true;

	#ifndef ROVER_INIT_VALUES
	#define ROVER_INIT_Values

	/* Publication of uORB messages */
	uORB::Publication<vehicle_control_mode_s>		_control_mode_pub{ORB_ID(vehicle_control_mode)};
	struct vehicle_control_mode_s				_control_mode {};

	uORB::Publication<vehicle_attitude_setpoint_s>		_att_sp_pub{ORB_ID(vehicle_attitude_setpoint)};
	struct vehicle_attitude_setpoint_s			_att_sp {};

	/* Publication of uORB messages */
	struct safety_s safety;
	uORB::Subscription safety_sub{ORB_ID(safety)};		// Safety switch request for starting and stopping the racing
	safety_sub.copy(&safety);

	/* Return motor control variables */
	roverControl motorControl;

	/* Start condition of the race */
	//bool start = 0;		// Create your own start condition

	/* Pixy2 Instance */
	Pixy2 pixy;
	bool wait = 1;		// needed for waiting for valid data
	usleep(5000);		// give pixy time to init
	#endif


	if (pixy.init() == 0) {

		pixy.getVersion();
		pixy.version->print();
		usleep(1000);

		while (1) {
			safety_sub.copy(&safety);				// request Safety swutch state
			safety.safety_off = 1;

			if (race) {
				pixy.line.getAllFeatures(LINE_VECTOR, wait);
				motorControl = raceTrack(pixy);
			} else {
				motorControl = {steer, speed};
				pixy.setLED(0,0,0);
			}

			_control_mode.flag_control_manual_enabled 	= false;
			_control_mode.flag_control_attitude_enabled 	= true;
			_control_mode.flag_control_velocity_enabled 	= false;
			_control_mode.flag_control_position_enabled	= false;

			roverSteerSpeed(motorControl, _att_sp);		// setting values for speed and steering to attitude setpoints

			// Publishing all
			_control_mode.timestamp = hrt_absolute_time();
			_control_mode_pub.publish(_control_mode);
			_att_sp.timestamp = hrt_absolute_time();
			_att_sp_pub.publish(_att_sp);

			if (threadShouldExit) {
				threadIsRunning = false;
				// reset speed and steering
				roverSteerSpeed(motorControl, _att_sp);
				// puplishing attitude setpoints
				_att_sp.timestamp = hrt_absolute_time();
				_att_sp_pub.publish(_att_sp);

				// Setting vehicle into the default state
				_control_mode.flag_control_manual_enabled 	= true;
				_control_mode.flag_control_attitude_enabled 	= true;
				_control_mode.flag_control_velocity_enabled 	= true;
				_control_mode.flag_control_position_enabled	= true;
				_control_mode.timestamp = hrt_absolute_time();
				_control_mode_pub.publish(_control_mode);

				PX4_INFO("Exit Rover Thread!\n");
				return 1;
			}
		}
	}
	return 0;
}


extern "C" __EXPORT int nxpcup_main(int argc, char *argv[]);
int nxpcup_main(int argc, char *argv[])
{

	if (argc < 2) {
		PX4_WARN("usage: nxpcup {start|stop|status}\n");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (threadIsRunning) {
			PX4_INFO("already running\n");
			/* this is not an error */
			return 0;
		}

		threadShouldExit = false;
		daemon_task = px4_task_spawn_cmd("nxpcup",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX - 5,
						 2000,
						 race_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)nullptr);

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		threadShouldExit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (threadIsRunning) {
			PX4_INFO("is running\n");

		} else {
			PX4_INFO("not started\n");
		}

		return 0;
	}

	if (!strcmp(argv[1], "aft")) {
		angle_for_turn = (double) atof(argv[2]);
		PX4_INFO("done\n");
		return 0;
	}

	if (!strcmp(argv[1], "cora")) {
		correction_angle = (double) atof(argv[2]);
		PX4_INFO("done\n");
		return 0;
	}

	if (!strcmp(argv[1], "speed")) {
		speed = atof(argv[2]);
		PX4_INFO("done\n");
		return 0;
	}

	if (!strcmp(argv[1], "tspeed")) {
		turn_speed = atof(argv[2]);
		PX4_INFO("done\n");
		return 0;
	}

	if (!strcmp(argv[1], "sspeed")) {
		straight_speed = atof(argv[2]);
		PX4_INFO("done\n");
		return 0;
	}

	if (!strcmp(argv[1], "steer")) {
		steer = atof(argv[2]);
		PX4_INFO("done\n");
		return 0;
	}

	if (!strcmp(argv[1], "kfa")) {
		kf_angle = (double) atof(argv[2]);
		PX4_INFO("done\n");
		return 0;
	}

	if (!strcmp(argv[1], "aft")) {
		angle_for_turn = atof(argv[2]);
		PX4_INFO("done\n");
		return 0;
	}

	if (!strcmp(argv[1], "maxc")) {
		max_counter = atoi(argv[2]);
		PX4_INFO("done\n");
		return 0;
	}

	if (!strcmp(argv[1], "race-start")) {
		race = true;
		PX4_INFO("done\n");
		return 0;
	}

	if (!strcmp(argv[1], "race-stop")) {
		speed = 0;
		steer = 0;
		race = false;
		PX4_INFO("done\n");
		return 0;
	}

	PX4_WARN("usage: race {start|stop|status}\n");
	return 1;
}

#define SCREEN_WIDTH 78
#define SCREEN_HEIGHT 51

void findLongestVectors(Vector* all_vectors, int all_vectors_count, int vectors_on_sides_count, Vector* result, int* count_result) {
	float max_left_length = 0, max_right_length = 0;
	for (int i = 0; i < all_vectors_count; i++) {
		Vector vector = all_vectors[i];
		int center = SCREEN_WIDTH / 2;
		float length = lengthVector(&vector);
		if (vector.m_x0 <= center) {
			if (max_left_length < length) {
				result[0] = vector;
				max_left_length = length;
			}
		} else {
			if (max_right_length < length) {
				result[1] = vector;
				max_right_length = length;
			}
		}
	}
	if (all_vectors_count > 0) {
		if (max_left_length <= 0 || max_right_length <= 0) {
			*count_result = 1;
		} else {
			*count_result = 2;
		}
	}
}

void sum_vectors(Vector* vectors, int count) {
	Vector result = vectors[0];
	sortVector(&result);
	for (int i = 1; i < count; i++) {
		Vector vector = vectors[i];
		sortVector(&vector);
		result.m_x0 += vector.m_x0;
		result.m_x1 += vector.m_x1;
		result.m_y0 += vector.m_y0;
		result.m_y1 += vector.m_y1;
	}
	result.m_x0 /= count;
	result.m_x1 /= count;
	result.m_y0 /= count;
	result.m_y1 /= count;
}

double calculateSteerAngle(Vector& vector, Pixy2& pixy) {
	return (vector.m_x1 < vector.m_x0 ? -kf_angle : kf_angle) * (90.0 - calculateAngle(&vector));
}

double calculateSteerAngleForStraight(Vector& vector, int count, Pixy2& pixy) {
	double angle = (count == 2 ? -1 : 1);
	int center = SCREEN_WIDTH / 2;
	if (vector.m_x0 <= center - 1) {
		angle *= correction_angle;
	} else if (vector.m_x0 >= center + 1) {
		angle *= -correction_angle;
	}
	return angle;
}

double calculateSteer(double steerAngle) {
	return steerAngle / 90.0;
}

roverControl raceTrack(Pixy2 &pixy) {
	roverControl control{steer, speed};
	if (pixy.line.numVectors == 0) {
		return control;
	}
	if (counter >= max_counter) {
		counter = 0;
		int count = 2;
		Vector* vectors = new Vector[count];
		findLongestVectors(pixy.line.vectors, pixy.line.numVectors, 1, vectors, &count);
		sum_vectors(vectors, count);
		Vector result = pixy.line.vectors[0];
		double steerAngle = calculateSteerAngle(result, pixy);
		if (abs(steerAngle) >= angle_for_turn && count == 1) {
			PX4_INFO("TURN: %f   %d", steerAngle, count);
			speed = turn_speed;
			steer = calculateSteer(steerAngle);
		} else {
			steerAngle = calculateSteerAngleForStraight(result, count, pixy);
			PX4_INFO("STRAIGHT: %f  %d", steerAngle, count);
			speed = straight_speed;
			steer = calculateSteer(steerAngle);
		}
		control.steer = steer;
		control.speed = speed;
		PX4_INFO("%f", (double) steer);
		PX4_INFO("");
	} else {
		counter++;
	}
	return control;
}


double lengthVector(Vector* vector) {
	return sqrt(pow(vector->m_x1 - vector->m_x0, 2) + pow(vector->m_y1 - vector->m_y0, 2));
}

double calculateAngle(Vector* vector) {
	return abs(asin(((double)vector->m_y1 - vector->m_y0) / lengthVector(vector)) * (180.0 / 3.14159));
}

void sortVector(Vector* vector) {
	if (vector->m_y0 < vector->m_y1) {
		double a = vector->m_x0;
		vector->m_x0 = vector->m_x1;
		vector->m_x1 = a;
		a = vector->m_y0;
		vector->m_y0 = vector->m_y1;
		vector->m_y1 = a;
	}
}
