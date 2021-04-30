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

int findVector(Pixy2 &pixy, Vector* left, Vector* right);
void findAverageVector(Vector* result, Vector* first, Vector* second);
float lengthVector(Vector* vector);
float calculateAngle(Vector* vector);
float calculateDiffAngleVector(double ref, Vector* vector);
roverControl raceTrack(Pixy2 &pixy);
void sortVector(Vector* vector);

//using namespace matrix;

const int NULL_VECTORS = 0;
const int LEFT_VECTOR = 1;
const int RIGHT_VECTOR = 2;
const int LEFT_AND_RIGHT_VECTOR = 3;

static int daemon_task;             /* Handle of deamon task / thread */

bool threadShouldExit = false;
bool threadIsRunning = false;
int counter = 0;

float speed = 0.15;
float steer = 0.0;
float correction_angle = 0.2;
float turn_angle = 0.7;
float angle_for_turn = 70.0;
float max_servo_angle = 0.7;
int max_counter = 30;

int debug = 0;

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
			pixy.line.getAllFeatures(LINE_VECTOR, wait);		// get line vectors from pixy

//			switch (safety.safety_off) {
//			case 0:
//				// Setting vehicle into the default state
//				_control_mode.flag_control_manual_enabled	= true;
//				_control_mode.flag_control_attitude_enabled	= true;
//				_control_mode.flag_control_velocity_enabled	= true;
//				_control_mode.flag_control_position_enabled	= true;
//
//				pixy.setLED(0,0,0);		// Pixy: reset RGB led
//				pixy.setLamp(false,false);	// Pixy: reset upper leds
//				// reset PWM outputs
//				motorControl.speed = 0.0f;
//				motorControl.steer = 0.0f;
//				break;
//			case 1:
//				// Setting vehicle to attitude control mode
//				_control_mode.flag_control_manual_enabled 	= false;
//				_control_mode.flag_control_attitude_enabled 	= true;
//				_control_mode.flag_control_velocity_enabled 	= false;
//				_control_mode.flag_control_position_enabled	= false;
//
//				start = true;			// create your own start condition
//				pixy.setLED(0,0,255);		// Pixy: set RGB led to blue
//				pixy.setLamp(true,false);	// Pixy: sets upper led
//
//				if (start) {
//					//motorControl = raceTrack(pixy);
//				} else {
//					motorControl.speed = speed;
//					motorControl.steer = steer;
//				}
//				break;
//			}

			motorControl = raceTrack(pixy);

			_control_mode.flag_control_manual_enabled 	= false;
			_control_mode.flag_control_attitude_enabled 	= true;
			_control_mode.flag_control_velocity_enabled 	= false;
			_control_mode.flag_control_position_enabled	= false;
			pixy.setLamp(true,false);

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

	if (!strcmp(argv[1], "speed")) {
			speed = atof(argv[2]);
			return 0;
	}

	if (!strcmp(argv[1], "steer")) {
			steer = atof(argv[2]);
			return 0;
	}

	if (!strcmp(argv[1], "msa")) {
		max_servo_angle = atof(argv[2]);
		return 0;
	}

	if (!strcmp(argv[1], "aft")) {
		angle_for_turn = atof(argv[2]);
		return 0;
	}

//	if (!strcmp(argv[1], "rab")) {
//		//ref_angle = atof(argv[2]);
//		return 0;
//	}

	if (!strcmp(argv[1], "debug")) {
		debug = atoi(argv[2]);
		return 0;
	}

	if (!strcmp(argv[1], "corra")) {
		correction_angle = atof(argv[2]);
		return 0;
	}

	if (!strcmp(argv[1], "turna")) {
		turn_angle = atof(argv[2]);
		return 0;
	}

	if (!strcmp(argv[1], "maxc")) {
		max_counter = atoi(argv[2]);
		return 0;
	}

	PX4_WARN("usage: race {start|stop|status}\n");
	return 1;
}

roverControl raceTrack(Pixy2 &pixy)
{
	/* insert you algorithm here */
	Vector left;
	Vector right;
	roverControl control{steer, speed};
	int find = findVector(pixy, &left, &right);
	float angle = 0;
	if (find == LEFT_VECTOR || find == RIGHT_VECTOR) {
		Vector vector = left;
		if (find == RIGHT_VECTOR) {
			vector = right;
		}
		if (calculateAngle(&vector) > angle_for_turn) {
			int x = vector.m_x0;
			if (x < 38) {
				angle = -correction_angle;
			} else if (x > 40) {
				angle = correction_angle;
			} else {
				angle = 0;
			}
		} else {
			angle = (find == RIGHT_VECTOR ? -1 : 1) * turn_angle;
		}
		control.steer = fmin(max_servo_angle, fmax(-max_servo_angle, angle));
		steer = control.steer;
		counter = 0;
	} else if (find == LEFT_AND_RIGHT_VECTOR && (abs(steer) >= turn_angle || counter++ >= max_counter)) {
		Vector average;
		findAverageVector(&average, &left, &right);
		int x = average.m_x0;
		if (x < 38) {
			angle = correction_angle;
		} else if (x > 40) {
			angle = -correction_angle;
		} else {
			angle = 0;
		}
		control.steer = fmin(max_servo_angle, fmax(-max_servo_angle, angle));
		steer = control.steer;
	}
	PX4_INFO("%f /n", (double) control.steer);
	return control;
}

int findVector(Pixy2 &pixy, Vector* left, Vector* right) {
	int max_length_left = 0;
	int max_length_right = 0;
	int find = NULL_VECTORS;
	for (int i = 0; i < pixy.line.numVectors; i++) {
		Vector vector = pixy.line.vectors[i];
		sortVector(&vector);
		int length = lengthVector(&vector);
		if (vector.m_x0 == 39) {
			if (vector.m_x1 > 39 && length > max_length_left) {
				max_length_left = length;
				*left = vector;
				if (find == NULL_VECTORS) {
					find = LEFT_VECTOR;
				} else if (find == RIGHT_VECTOR) {
					find = LEFT_AND_RIGHT_VECTOR;
				}
			} else if (vector.m_x1 < 39 && length > max_length_right) {
				max_length_right = length;
				*right = vector;
				if (find == NULL_VECTORS) {
					find = RIGHT_VECTOR;
				} else if (find == LEFT_VECTOR) {
					find = LEFT_AND_RIGHT_VECTOR;
				}
			}
		} else if (vector.m_x0 < 39 && length > max_length_left) {
			max_length_left = length;
			*left = vector;
			if (find == NULL_VECTORS) {
				find = LEFT_VECTOR;
			} else if (find == RIGHT_VECTOR) {
				find = LEFT_AND_RIGHT_VECTOR;
			}
		} else if (length > max_length_right) {
			max_length_right = length;
			*right = vector;
			if (find == NULL_VECTORS) {
				find = RIGHT_VECTOR;
			} else if (find == LEFT_VECTOR) {
				find = LEFT_AND_RIGHT_VECTOR;
			}
		}
	}

	if (find == LEFT_AND_RIGHT_VECTOR) {
		Vector main;
		Vector second;
		int local_find;
		if (left->m_y0 < right->m_y0) {
			main = *right;
			second = * left;
			local_find = RIGHT_VECTOR;
		} else if (right->m_y0 < left->m_y0) {
			main = *left;
			second = * right;
			local_find = LEFT_VECTOR;
		} else {
			return find;
		}
		double k = (main.m_y0 - main.m_y1) / (main.m_x0 - main.m_x1);
		double b = main.m_y1 - k * main.m_x1;

//		double local_y = (second.m_x0 - main.m_x0) * kf;
//		double y = (int) ((main.m_x0 + local_y + 0.01) * 100) / 100;
		if ((second.m_x0 * k + b ) > second.m_y0) {
			main.print();
			second.print();
			PX4_INFO("%f %f %f", k, b, second.m_x0 * k + b);
			return local_find;
		}
	}


	return find;
}

void findAverageVector(Vector* result, Vector* first, Vector* second) {
	result->m_x0 = (first->m_x0 + second->m_x0) / 2;
	result->m_y0 = (first->m_y0 + second->m_y0) / 2;
	result->m_x1 = (first->m_x1 + second->m_x1) / 2;
	result->m_y1 = (first->m_y1 + second->m_y1) / 2;
}

float lengthVector(Vector* vector) {
	return sqrt(pow(vector->m_x1 - vector->m_x0, 2) + pow(vector->m_y1 - vector->m_y0, 2));
}

float calculateAngle(Vector* vector) {
	return abs(asin(((float)vector->m_y1 - vector->m_y0) / lengthVector(vector)) * (180.0 / 3.14159));
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
