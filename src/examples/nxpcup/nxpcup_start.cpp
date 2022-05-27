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

#define PI 3.14159265f

#define TO_LEFT -1
#define TO_RIGHT 1
#define FORWARD 0
#define LEFT_UP 0
#define LEFT_DOWN 1
#define RIGHT_UP 2
#define RIGHT_DOWN 3

#define SCREEN_WIDTH 78
#define HALF_SCREEN_WIDTH SCREEN_WIDTH/2
#define SCREEN_HEIGHT 51
#define HALF_SCREEN_HEIGHT SCREEN_HEIGHT/2

typedef float Steer;
typedef float Angle;
typedef float Speed;

static int daemon_task;             /* Handle of deamon task / thread */

bool threadShouldExit = false;
bool threadIsRunning = false;
int speed_counter = 0;

Speed min_speed = 0.15;
Speed max_speed = 0.15;

//скорость на которой постоянно ездит машина
Speed speed = 0.0;

//последний угол поворота
Steer steer = 0.0;
Angle angle_for_turn = 35.0;
Angle correction_angle = 20;
Angle kf_angle = 1.8;
int max_speed_counter = 10;
bool race = true;
int debug = 0;

struct Point;
struct ExpandVector;

int getSide(Vector& vector);
void reverseVectorIfNeeded(Vector& vector);
float lengthVector(Vector& vector);
void raceTrack(Pixy2 &pixy);
Angle calculateAngle(ExpandVector& vector);

struct Point {
	int x;
	int y;
	int position;

	void calculatePosition() {
		position += x > HALF_SCREEN_WIDTH;
		position += y > HALF_SCREEN_HEIGHT;
	}
};

struct ExpandVector {
	Point down = Point{0, 0};
	Point up = Point{0, 0};
	float length = 0.0f;
	int side = 0;
	bool isNull = true;
	bool isBad = false;

	bool canUse() {
		return !isNull && !isBad;
	}

	void set(Vector vector) {
		reverseVectorIfNeeded(vector);
		down.x = vector.m_x0;
		down.y = vector.m_y0;
		down.calculatePosition();
		up.x = vector.m_x1;
		up.y = vector.m_y1;
		up.calculatePosition();
		isNull = false;
	}

	void calculateSide() {
		side = down.x < up.x ? TO_RIGHT : down.x > up.x ? TO_LEFT : FORWARD;
	}
};


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
				pixy.setLamp(true, true);
				pixy.line.getAllFeatures(LINE_VECTOR, wait);
				raceTrack(pixy);
			} else {
				pixy.setLED(0,0,0);
			}

			_control_mode.flag_control_manual_enabled 	= false;
			_control_mode.flag_control_attitude_enabled 	= true;
			_control_mode.flag_control_velocity_enabled 	= false;
			_control_mode.flag_control_position_enabled	= false;

			motorControl = {steer, speed};
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
		min_speed = speed;
		max_speed = speed;
		PX4_INFO("done\n");
		return 0;
	}

	if (!strcmp(argv[1], "tspeed")) {
		min_speed = atof(argv[2]);
		PX4_INFO("done\n");
		return 0;
	}

	if (!strcmp(argv[1], "sspeed")) {
		max_speed = atof(argv[2]);
		PX4_INFO("done\n");
		return 0;
	}

	if (!strcmp(argv[1], "steer")) {
		steer = atof(argv[2]);
		PX4_INFO("done\n");
		return 0;
	}

	if (!strcmp(argv[1], "kfa")) {
		kf_angle = atof(argv[2]);
		PX4_INFO("done\n");
		return 0;
	}

	if (!strcmp(argv[1], "aft")) {
		angle_for_turn = atof(argv[2]);
		PX4_INFO("done\n");
		return 0;
	}

	if (!strcmp(argv[1], "msc")) {
		max_speed_counter = atoi(argv[2]);
		PX4_INFO("done\n");
		return 0;
	}

	if (!strcmp(argv[1], "debug")) {
		debug = atoi(argv[2]);
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

//технические функции
int getSide(Vector& vector) {
	return vector.m_x1 < vector.m_x0 ? TO_LEFT : vector.m_x1 > vector.m_x0 ? TO_RIGHT : FORWARD;
}

float lengthVector(Vector& vector) {
	return sqrt(pow(vector.m_x1 - vector.m_x0, 2) + pow(vector.m_y1 - vector.m_y0, 2));
}

Angle calculateAngle(ExpandVector& vector) {
	return abs((float) asin(((float)vector.up.y - vector.down.y) / vector.length) * (180.0f / PI));
}

void reverseVectorIfNeeded(Vector& vector) {
	if (vector.m_y0 < vector.m_y1) {
		double a = vector.m_x0;
		vector.m_x0 = vector.m_x1;
		vector.m_x1 = a;
		a = vector.m_y0;
		vector.m_y0 = vector.m_y1;
		vector.m_y1 = a;
	}
}

void sum_vectors(ExpandVector vectors[], int count) {
	ExpandVector& result = vectors[0];
	for (int i = 1; i < count; i++) {
		ExpandVector vector = vectors[i];
		if (vector.isNull) {
			continue;
		}
		//PX4_INFO("%d  %d  %d  %d", vector.m_x0, vector.m_y0, vector.m_x1, vector.m_y1);
		result.down.x += vector.down.x;
		result.down.y += vector.down.y;
		result.up.x += vector.up.x;
		result.up.y += vector.up.y;
		result.isBad += result.isBad;
	}
	result.down.x /= count;
	result.down.y /= count;
	result.up.x /= count;
	result.up.y /= count;
	result.calculateSide();
	result.down.calculatePosition();
	result.up.calculatePosition();
	//PX4_INFO("");
}

//функции фильтрации
bool isBadVector(Vector vector) {
	int side = getSide(vector);
	if (vector.m_x0 <= HALF_SCREEN_WIDTH && side == TO_LEFT) {
		return true;
	} else if (vector.m_x0 >= HALF_SCREEN_WIDTH && side == TO_RIGHT) {
		return true;
	}
	return false;
}

void findLongestVectorsByFourSectors(Vector* all_vectors, int all_vectors_count, ExpandVector result[], int* count_result) {
	int n = *count_result;
	for (int i = 0; i < n; i++) {
		result[i] = ExpandVector{};
	}
	PX4_INFO("start");
	float max_left_upper_length = 0, max_left_lower_length = 0, max_right_upper_length = 0, max_right_lower_length = 0;
	PX4_INFO("vectors:  ");
	for (int i = 0; i < all_vectors_count; i++) {
		Vector vector = all_vectors[i];
		reverseVectorIfNeeded(vector);
		PX4_INFO("%d  %d  %d  %d", vector.m_x0, vector.m_y0, vector.m_x1, vector.m_y1);
		float length = lengthVector(vector);
		if (vector.m_x0 <= HALF_SCREEN_WIDTH) {
			if (vector.m_y0 <= HALF_SCREEN_HEIGHT) {
				if (max_left_upper_length < length) {
					ExpandVector& newVector = result[LEFT_UP];
					newVector.set(vector);
					newVector.length = length;
					newVector.calculateSide();
					newVector.isBad = isBadVector(vector);
					max_left_upper_length = length;
				}
			} else {
				ExpandVector& newVector = result[LEFT_DOWN];
				if (max_left_lower_length < length) {
					newVector.set(vector);
					newVector.length = length;
					newVector.calculateSide();
					newVector.isBad = isBadVector(vector);
					max_left_lower_length = length;
				}
			}
		} else {
			if (vector.m_y0 <= HALF_SCREEN_HEIGHT) {
				if (max_right_upper_length < length) {
					ExpandVector& newVector = result[RIGHT_UP];
					newVector.set(vector);
					newVector.length = length;
					newVector.calculateSide();
					newVector.isBad = isBadVector(vector);
					max_right_upper_length = length;
				}
			} else {
				ExpandVector& newVector = result[RIGHT_DOWN];
				if (max_right_lower_length < length) {
					newVector.set(vector);
					newVector.length = length;
					newVector.calculateSide();
					newVector.isBad = isBadVector(vector);
					max_right_lower_length = length;
				}
			}
		}
	}
	if (debug > 3) {
		PX4_INFO("");
		PX4_INFO("findLongestVectorsByFourSectors:  ");
		for (int i = 0; i < *count_result; i++) {
			ExpandVector vector = result[i];
			if (vector.isNull) {
				PX4_INFO("null");
			} else {
				PX4_INFO("%d  %d  %d  %d    %s", vector.down.x, vector.down.y, vector.up.x, vector.up.y, (vector.isBad) ? "bad" : "okey");
			}
		}
		PX4_INFO("");
	}
}

void chooseVector(ExpandVector& top, ExpandVector& down, ExpandVector output[]) {
	output[0] = down.isNull ? top : down;
}

void chooseFromLongestVectors(ExpandVector vectors[], int* count) {
	int id = -1;
	ExpandVector output[1];

	bool found = false;
	ExpandVector down{};
	if (vectors[LEFT_DOWN].canUse() && vectors[RIGHT_DOWN].canUse()) {
		down = (vectors[LEFT_DOWN].down.y > vectors[RIGHT_DOWN].down.y) ? vectors[LEFT_DOWN] : vectors[RIGHT_DOWN];
		found = true;
	} else if (vectors[RIGHT_DOWN].canUse()) {
		down = vectors[RIGHT_DOWN];
		found = true;
	} else if (vectors[LEFT_DOWN].canUse()) {
		down = vectors[LEFT_DOWN];
		found = true;
	}
	if (found) {
		for (int i = 0; i < *count; i++) {
			if (vectors[i].down.y <= down.up.y) {
				vectors[i].isNull = true;
			}
		}

	}

	chooseVector(vectors[LEFT_UP], vectors[LEFT_DOWN], output);
	ExpandVector left = output[0];
	chooseVector(vectors[RIGHT_UP], vectors[RIGHT_DOWN], output);
	ExpandVector right = output[0];
	*count = 0;
	id = 0;
	if (!left.isNull) {
		*count += 1;
		vectors[id++] = left;
	}
	if (!right.isNull)  {
		*count += 1;
		vectors[id] = right;
	}
	if (debug > 2) {
		PX4_INFO("chooseFromLongestVectors:  ");
		for (int i = 0; i < *count; i++) {
			ExpandVector vector = vectors[i];
			if (vector.isNull) {
				PX4_INFO("null");
			} else {
				PX4_INFO("%d  %d  %d  %d    %s", vector.down.x, vector.down.y, vector.up.x, vector.up.y, (vector.isBad) ? "bad" : "okey");
			}
		}
		PX4_INFO("");
	}
}

//
bool isTurnAngle(Angle steerAngle) {
	return abs(steerAngle) >= angle_for_turn;
}

Angle transponireAngle(Angle angle) {
	return 90.0f - angle;
}

Angle calculateSteerAngleForTurn(ExpandVector& vector, Angle kfa, Pixy2& pixy) {
	return vector.side * (kfa * transponireAngle(calculateAngle(vector)));
}

Angle calculateSteerAngleForStraight(ExpandVector& vector, int count, Pixy2& pixy) {
	if (count == 1 && vector.isBad) {
		return 0;
	}
	Angle angle = (count >= 2 ? -1 : 1);
	int center  = 10;
	if (vector.down.x < HALF_SCREEN_WIDTH - center) {
		angle *= correction_angle;
	} else if (vector.down.x > HALF_SCREEN_WIDTH + center) {
		angle *= -correction_angle;
	} else if (count == 1) {
		angle = vector.side * 60.0f;
	} else {
		angle = 0;
	}
	return angle;
}

Steer calculateSteer(Angle steerAngle) {
	return steerAngle / 90.0;
}

void resetSpeed() {
	speed = min_speed;
	speed_counter = 0;
}

Speed calcutateSpeed() {
	if (speed_counter < max_speed_counter) {
		speed_counter++;
	}
	return min_speed + ((max_speed - min_speed) / max_speed_counter * speed_counter);
}

void raceTrack(Pixy2 &pixy) {
	if (pixy.line.numVectors == 0) {
		return;
	}
	int count = 4;
	ExpandVector vectors[4];
	findLongestVectorsByFourSectors(pixy.line.vectors, pixy.line.numVectors, vectors, &count);
	chooseFromLongestVectors(vectors, &count);
	if (count > 0) {
		if (count > 1) {
			sum_vectors(vectors, count);
		}
		ExpandVector average = vectors[0];
		Angle angle = transponireAngle(calculateAngle(average));
		Angle steerAngle;
		if (isTurnAngle(angle) && count == 1 && !average.isBad) {
			steerAngle = calculateSteerAngleForTurn(average, kf_angle + ((average.side == TO_LEFT) ? 0.2f : 0), pixy);
			PX4_INFO("TURN: %f   %d", (double) steerAngle, count);
			resetSpeed();
			steer = calculateSteer(steerAngle);
		} else {
			steerAngle = calculateSteerAngleForStraight(average, count, pixy);
			PX4_INFO("STRAIGHT: %f  %d", (double) steerAngle, count);
			speed = calcutateSpeed();
			steer = calculateSteer(steerAngle);
		}
		PX4_INFO("vector angle:  %f", (double) angle);
		PX4_INFO("average:  %d  %d  %d  %d    %s", average.down.x, average.down.y, average.up.x, average.up.y, (average.isBad) ? "bad" : "okey");
	} else {
		resetSpeed();
	}
	PX4_INFO("steer:   %f", (double) steer);
	PX4_INFO("speed:   %f", (double) speed);
	PX4_INFO("");
}
