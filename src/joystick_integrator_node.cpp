/**
 * \file joystick_integrator.cpp
 * \brief
 *
 * \author Andrew Price
 * \date 7 4, 2013
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

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

ros::Subscriber joySub;
ros::Publisher posePub;
sensor_msgs::Joy prevJoy;
geometry_msgs::PoseStamped curPose;
Eigen::Quaternionf curOrientation;

bool useDominantAxis = true;
bool useLocalCoordinates = false;
double positionSpeed = 0.5;
double rotationSpeed = 0.5;

const int X_AXIS = 0;
const int Y_AXIS = 1;
const int Z_AXIS = 2;
const int RX_AXIS = 3;
const int RY_AXIS = 4;
const int RZ_AXIS = 5;

Eigen::Quaternionf rpyToQ (float rx, float ry, float rz)
{
	Eigen::Quaternion<float> q;

	Eigen::AngleAxis<float> aaZ(rz, Eigen::Vector3f::UnitZ());
	Eigen::AngleAxis<float> aaY(ry, Eigen::Vector3f::UnitY());
	Eigen::AngleAxis<float> aaX(rx, Eigen::Vector3f::UnitX());

	q = aaZ * aaY * aaX;

	return q;
}

// [x,y,z,r,p,y]
void spacenavCallback(const sensor_msgs::JoyPtr joy)
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
	if (useDominantAxis)
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



	Eigen::Quaternionf q = rpyToQ(joy->axes[RX_AXIS] * dt * rotationSpeed,
								  joy->axes[RY_AXIS] * dt * rotationSpeed,
								  joy->axes[RZ_AXIS] * dt * rotationSpeed);

	if (useLocalCoordinates)
	{
		// Perform translation in direction of current axes
		Eigen::Vector3f dp(joy->axes[X_AXIS] * dt * positionSpeed,
						   joy->axes[Y_AXIS] * dt * positionSpeed,
						   joy->axes[Z_AXIS] * dt * positionSpeed);

		dp = curOrientation * dp;

		curPose.pose.position.x += dp.x();
		curPose.pose.position.y += dp.y();
		curPose.pose.position.z += dp.z();

		// Perform rotation in terms of current axes
		curOrientation = curOrientation * q;
	}
	else
	{
		// Perform translation in terms of global axes
		curPose.pose.position.x += joy->axes[X_AXIS] * dt * positionSpeed;
		curPose.pose.position.y += joy->axes[Y_AXIS] * dt * positionSpeed;
		curPose.pose.position.z += joy->axes[Z_AXIS] * dt * positionSpeed;

		// Perform rotation in terms of global axes
		curOrientation = q * curOrientation;
	}

	curPose.pose.orientation.w = curOrientation.w();
	curPose.pose.orientation.x = curOrientation.x();
	curPose.pose.orientation.y = curOrientation.y();
	curPose.pose.orientation.z = curOrientation.z();

	curPose.header.frame_id = "/Body_Hip";
	curPose.header.stamp = ros::Time::now();

	posePub.publish(curPose);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "joystick_integrator");
	ROS_INFO("Started joystick_integrator.");

	ros::NodeHandle nh;

	nh.param("/use_dominant_axis", useDominantAxis, true);
	nh.param("/use_local_coordinates", useLocalCoordinates, false);
	nh.param("/position_speed", positionSpeed, 0.25);
	nh.param("/rotation_speed", rotationSpeed, 0.25);

	curOrientation = Eigen::Quaternionf(Eigen::Matrix3f::Identity());

	posePub = nh.advertise<geometry_msgs::PoseStamped>("/pose_out", 1);
	joySub = nh.subscribe("/joy_in", 1, spacenavCallback);


	ros::spin();

	return 0;
}
