/**
 * \file JoystickIntegrator.cpp
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

#include "joystick_integrator/JoystickIntegrator.h"

Eigen::Quaternionf rpyToQ (float rx, float ry, float rz)
{
	Eigen::Quaternion<float> q;

	Eigen::AngleAxis<float> aaZ(rz, Eigen::Vector3f::UnitZ());
	Eigen::AngleAxis<float> aaY(ry, Eigen::Vector3f::UnitY());
	Eigen::AngleAxis<float> aaX(rx, Eigen::Vector3f::UnitX());

	q = aaZ * aaY * aaX;

	return q;
}

JoystickIntegratorParameters JoystickIntegratorParameters::defaults()
{
	JoystickIntegratorParameters p;
	p.useDominantAxis = true;
	p.useLocalRotations = true;
	p.useLocalTranslations = false;
	p.positionSpeed = 0.1;
	p.rotationSpeed = 0.2;

	return p;
}

std::ostream& operator<<(std::ostream& stream, const JoystickIntegratorParameters& p)
{
	stream << "Dominant Axis: " << p.useDominantAxis << std::endl;
	stream << "Local Rotations: " << p.useLocalRotations << std::endl;
	stream << "Local Translations: " << p.useLocalTranslations << std::endl;
	stream << "Rotation Speed: " << p.rotationSpeed << std::endl;
	stream << "Translation Speed: " << p.positionSpeed << std::endl;

	return stream;
}

JoystickIntegrator::JoystickIntegrator(std::string baseFrame)
{
	frame_id = baseFrame;
	currentOrientation = Eigen::Quaternionf(Eigen::Matrix3f::Identity());
	currentPose.header.frame_id = frame_id;
	currentPose.pose.position.x = 0;
	currentPose.pose.position.y = 0;
	currentPose.pose.position.z = 0;
	currentPose.pose.orientation.x = 0;
	currentPose.pose.orientation.y = 0;
	currentPose.pose.orientation.z = 0;
	currentPose.pose.orientation.w = 1;
	settings = JoystickIntegratorParameters::defaults();
}

// [x,y,z,r,p,y]
void JoystickIntegrator::spacenavUpdate(const sensor_msgs::JoyPtr joy)
{
	double dt = 0.0;
	ros::Duration deltaT = (joy->header.stamp - prevJoy.header.stamp);

	// Check that the last message is still relatively recent, else clamp to 1 second interval
	if (deltaT > ros::Duration(1))
	{
		dt = 1.0;
	}
	else
	{
		dt = deltaT.toSec();
	}

	// Limit movement to the primary commanded direction.
	// This helps limit drift in unintended directions.
	if (settings.useDominantAxis)
	{
		int dominantAxis = 0;
		float max = 0;
		for (int i = 0; i < 6; i++)
		{
			if (fabs(joy->axes[i]) > max)
			{
				max = fabs(joy->axes[i]);
				dominantAxis = i;
			}
		}

		for (int i = 0; i < 6; i++)
		{
			if (i != dominantAxis)
			{
				joy->axes[i] = 0.0;
			}
		}
	}


	Eigen::Quaternionf q = rpyToQ(joy->axes[RX_AXIS] * dt * settings.rotationSpeed,
								  joy->axes[RY_AXIS] * dt * settings.rotationSpeed,
								  joy->axes[RZ_AXIS] * dt * settings.rotationSpeed);

	if (settings.useLocalTranslations)
	{
		// Perform translation in direction of current axes
		Eigen::Vector3f dp(joy->axes[X_AXIS] * dt * settings.positionSpeed,
						   joy->axes[Y_AXIS] * dt * settings.positionSpeed,
						   joy->axes[Z_AXIS] * dt * settings.positionSpeed);

		dp = currentOrientation * dp;

		currentPose.pose.position.x += dp.x();
		currentPose.pose.position.y += dp.y();
		currentPose.pose.position.z += dp.z();
	}
	else
	{
		// Perform translation in terms of global axes
		currentPose.pose.position.x += joy->axes[X_AXIS] * dt * settings.positionSpeed;
		currentPose.pose.position.y += joy->axes[Y_AXIS] * dt * settings.positionSpeed;
		currentPose.pose.position.z += joy->axes[Z_AXIS] * dt * settings.positionSpeed;
	}

	if (settings.useLocalRotations)
	{
		// Perform rotation in terms of current axes
		currentOrientation = currentOrientation * q;
	}
	else
	{
		// Perform rotation in terms of global axes
		currentOrientation = q * currentOrientation;
	}


	currentPose.pose.orientation.w = currentOrientation.w();
	currentPose.pose.orientation.x = currentOrientation.x();
	currentPose.pose.orientation.y = currentOrientation.y();
	currentPose.pose.orientation.z = currentOrientation.z();

	currentPose.header.frame_id = frame_id;
	currentPose.header.stamp = ros::Time::now();

	return;
}

void JoystickIntegrator::setPose(const geometry_msgs::PoseStamped& newPose)
{
	currentPose = newPose;
	currentOrientation.w() = newPose.pose.orientation.w;
	currentOrientation.x() = newPose.pose.orientation.x;
	currentOrientation.y() = newPose.pose.orientation.y;
	currentOrientation.z() = newPose.pose.orientation.z;
}
