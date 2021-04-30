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
 * @file hello_example.cpp
 * Race code for NXP Cup
 *
 * @author Katrin Moritz
 */

#include "nxpcup_race.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

int findVector(Pixy2 &pixy, Vector* first, Vector* second);
void findAverageVector(Vector* result, Vector* first, Vector* second);
double lengthVector(Vector* vector);
double calculateAngle(Vector* vector);

roverControl raceTrack(Pixy2 &pixy)
{
	roverControl control{};
	/* insert you algorithm here */
	Vector first;
	Vector second;
	int find = findVector(pixy, &first, &second);

	if (find == 1){
		double angle = calculateAngle(&first);
		control.steer = (first.m_x0 > first.m_x1 ? -1 : 1) * (65.0 / angle);
	} else if (find == 2) {
		Vector result;
		findAverageVector(&result, &first, &second);
		result.print();
	}
	PX4_INFO("\n");

	//pixy.line.vectors->print();
	//PX4_INFO();
	/*Vector current;
	if (!findVector(pixy, &current)) {
		if (abs(current.m_x0 - current.m_x1) > 100) {
			pixy.setLamp(false, false);
		} else {
			pixy.setLamp (false, true);
		}
	} else {
		pixy.setLED(5, 120, 5);
		pixy.setLamp(true, false);
	}*/
	return control;
}

int findVector(Pixy2 &pixy, Vector* first, Vector* second) {
	int max_length_1 = 0;
	int max_length_2 = 0;
	int find = false;
	for (int i = 0; i < pixy.line.numVectors; i++) {
		Vector vector = pixy.line.vectors[i];
		int sqare_length = lengthVector(&vector);
		if (max_length_1 < sqare_length) {
			if (max_length_1 != 0) {
				*second = *first;
				max_length_2 = max_length_1;
				if (find < 2) {
					find = 2;
				}
			}
			*first = vector;
			max_length_1 = sqare_length;
			find = true;
			if (find < 1) {
				find = 1;
			}
		} else if (max_length_2 < sqare_length) {
			*second = vector;
			max_length_2 = sqare_length;
			if (find < 2) {
				find = 2;
			}
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

double lengthVector(Vector* vector) {
	return sqrt(pow(vector->m_x1 - vector->m_x0, 2) + pow(vector->m_y1 - vector->m_y0, 2));
}

double calculateAngle(Vector* vector) {
	return asin(((double)vector->m_y1 - vector->m_y0) / lengthVector(vector));
}
