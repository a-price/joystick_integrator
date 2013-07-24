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

#include "joystick_integrator/JoystickIntegrator.h"

ros::Subscriber joySub;
ros::Publisher posePub;
JoystickIntegrator joyInt("/Body_TSY");

void spacenavCallback(const sensor_msgs::JoyPtr joy)
{
	joyInt.spacenavUpdate(joy);

	posePub.publish(joyInt.currentPose);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "joystick_integrator");
	ROS_INFO("Started joystick_integrator.");

	ros::NodeHandle nh;

	nh.param("/use_dominant_axis", joyInt.settings.useDominantAxis, true);
	nh.param("/use_local_rotations", joyInt.settings.useLocalRotations, true);
	nh.param("/use_local_translations", joyInt.settings.useLocalTranslations, false);
	nh.param("/position_speed", joyInt.settings.positionSpeed, 0.1);
	nh.param("/rotation_speed", joyInt.settings.rotationSpeed, 0.25);

	posePub = nh.advertise<geometry_msgs::PoseStamped>("/pose_out", 1);
	joySub = nh.subscribe("/joy_in", 1, spacenavCallback);


	ros::spin();

	return 0;
}
