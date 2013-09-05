/**
 * \file JoystickIntegrator.h
 * \brief
 *
 * \author Andrew Price
 * \date July 23, 2013
 *
 * \copyright
 *
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Humanoid Robotics Lab Georgia Institute of Technology
 * Director: Mike Stilman http://www.golems.org
 *
 * This file is provided under the following "BSD-style" License:
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef JOYSTICKINTEGRATOR_H
#define JOYSTICKINTEGRATOR_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

const int X_AXIS = 0;
const int Y_AXIS = 1;
const int Z_AXIS = 2;
const int RX_AXIS = 3;
const int RY_AXIS = 4;
const int RZ_AXIS = 5;

Eigen::Quaternionf rpyToQ (float rx, float ry, float rz);

class JoystickIntegratorParameters
{
public:
	bool useDominantAxis;
	bool useLocalRotations;
	bool useLocalTranslations;
	double positionSpeed;
	double rotationSpeed;

	static JoystickIntegratorParameters defaults();
};

static std::ostream& operator<<(std::ostream& stream, const JoystickIntegratorParameters& p);

class JoystickIntegrator
{
public:
	JoystickIntegrator(std::string baseFrame);
	~JoystickIntegrator() {}

	void spacenavUpdate(const sensor_msgs::JoyPtr joy);

    void setPose(const geometry_msgs::PoseStamped& newPose);

	JoystickIntegratorParameters settings;
	sensor_msgs::Joy prevJoy;
	geometry_msgs::PoseStamped currentPose;
	Eigen::Quaternionf currentOrientation;

	std::string frame_id;
};

#endif // JOYSTICKINTEGRATOR_H
