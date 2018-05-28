/*******************************************************************************
* Copyright (C) 2015-2020, idriverplus(BeiJing ZhiXingZhe, Inc.)
* 
* NodeName: ivsteercontrol
* FileName: steercontrol.h, steercontrol.cpp
* 
* Description: 
* 1. calculate the steering angle to track the target path
* 2. calculate the torque
*
* History: 
* jianwei         17/06/20    1.0.0    build this module. 
********************************************************************************/
#ifndef _STEERCONTROL_H
#define _STEERCONTROL_H
#pragma once
//c++ lib
#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <limits>
//ROS lib
#include "ros/ros.h"
#include "ros/time.h"
#include "math.h"
//#include "monitor_client.h" 

using namespace Eigen;


struct sNearPath
{
	VectorXd x;
	VectorXd y;
};

struct sCarCoord
{
	double x_car[4];
	double y_car[4];
};

struct sPredictPoint
{
	VectorXd x;
	VectorXd y;
	VectorXd psi;
	VectorXd crosserr;
	VectorXd psierr;
	VectorXd evalu;
	VectorXd epsdeg;
};

struct sThetaTargetNear
{
	double thetatargetnearmax;
	double thetatargetnearmin;
};

struct sPathCurrentVehicle
{
	double xleft;
	double yleft;
	double xright;
	double yright;
};

struct sTheAdjustDpreview
{
	double adjust_dpreview;
	double adjust_dpreview_near;
};

struct sArcReturn
{
	double radiusarc;
	double alpha1;
	double alpha2;
	double arc_center_x;
	double arc_center_y;
};

struct sPath
{
	VectorXd x;
	VectorXd y;
	VectorXd angle;
	VectorXd distance;
};

struct sPoint
{
	double x;
	double y;
	double angle;
};

struct sTrackError
{ 
	double lateral_err;
	double heading_err;
};

struct mean_filter_param
{
	std::vector<double> data;
	int window_size;	
};
 #endif  // _STEERCONTROL_H

