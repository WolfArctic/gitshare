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
* zhangzhuo        18/03/17    1.0.0    build this module. 
********************************************************************************/
#ifndef _WSTEERCONTROL_H
#define _WSTEERCONTROL_H

#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>

#include "ros/ros.h"
#include "ros/time.h"
#include "math.h"

#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"

#include "ivsteercontrol/ivsteercontrol.h"
#include "ivpathplanner/ivmsgpath.h"
#include "ivmap/ivmapmsgvap.h"
#include "steercontrol.h"
#include "filter.h"


class wsteercontrol
{

public:
	wsteercontrol(ros::NodeHandle mh);
	~wsteercontrol();

	void run();
	void StartDataProcess();
	void SubMotionplanner(const ivpathplanner::ivmsgpath::ConstPtr &msg);
	void SubVAP(const ivmap::ivmapmsgvap::ConstPtr &msg);	
	void Initial();
	void CalculateMain();
	void GetTargetPath();
	void AddDistanceToPath(sPath &smooth_path);
	int Signn(double value);
	int GetNearestPointNum(double x_point, double y_point);
	int NearEqual(double num1, double num2);
	double GetTargetAngleStanley(double lateral_error, double heading_error, 
						  double wheel_speed, double k_coe);
	double GetTargetAnglePureP(double reference_point_x, double reference_point_y,
	sCarCoord &vehicle_coordinate);
	double wheel2eps(double wheel_angle);
	double eps2wheel(double eps_angle);
	double GetLength(double x_point1, double y_point1, double x_point2, double y_point2);
	double GetDist2Seg(double x, double y, double x1, double y1, double x2, double y2);
	double LimiteTargetAngle(double original_angle);
	double GetLineDirection(double x_from, double y_from, double x_to, double y_to);
	sPath GetOriginalPath() const;
	sPath GetSmoothPath(sPath &original_path) const;
	sTrackError GetTrackError(int id_nearest_point, sCarCoord &car_position);
	sCarCoord GetCarCoordinate ();
private:
	int amount_motion_path_point;
	int amount_target_path_point;
	int sc_loop_freq;
	int path_end_flag;
	//car parameters
	double length_vehicle;
	double width_vehicle;
	double k_ratio_r;
	double k_ratio_l;
	double steerangle_max_r;
	double steerangle_max_l;
	double vehicle_angle_center;
	double gps_angle_center;
	double torque_max;	
	double x_middle_front;
	double y_middle_front;

	double actuator_steerangle;
	double actuator_speed;
	double DMAX_VALUE = std::numeric_limits<double>::max();

	double DMIN_VALUE = std::numeric_limits<double>::min();
	sPath target_path;

	ivpathplanner::ivmsgpath motionpath;
	ivsteercontrol::ivsteercontrol resultcontrol;	
	mean_filter_param car_speed_;
	mean_filter_param eps_angle_;
	mean_filter_param target_output_;
	Filter *filter = new Filter();

	ros::Subscriber    sub_;
	ros::Subscriber    sub2_;
	ros::Publisher     pub_;
};

#endif // _WSTEERCONTROL_H

