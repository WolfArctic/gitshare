#include "wsteercontrol.h"
 
 wsteercontrol::wsteercontrol(ros::NodeHandle mh)
 {
	sub_ = mh.subscribe("ivmotionplanner", 1000, &wsteercontrol::SubMotionplanner,this);
	sub2_ = mh.subscribe("ivmapvap", 1000, &wsteercontrol::SubVAP,this);
	pub_ = mh.advertise<ivsteercontrol::ivsteercontrol>("ivsteercontrol", 1000);
	Initial();    
 }

wsteercontrol::~wsteercontrol()
{
	if (filter != NULL){
		delete (filter);
	}
}

void wsteercontrol::Initial()
{
	ros::NodeHandle mh;
	amount_motion_path_point = 0;
	sc_loop_freq = 0;
	path_end_flag = 0;
	actuator_steerangle = 0;
	actuator_speed = 0;
	mh.param("scloopfrep",sc_loop_freq,sc_loop_freq);
	mh.param("lengthWheelBase",length_vehicle,length_vehicle);
	mh.param("widthVehicle",width_vehicle,width_vehicle);
	mh.param("reduceRatioLift",k_ratio_l,k_ratio_l);
	mh.param("reduceRatioRight",k_ratio_r,k_ratio_r);
	mh.param("steerAngleMaxLift", steerangle_max_l,steerangle_max_l);
	mh.param("steerAngleMaxRight",steerangle_max_r,steerangle_max_r);
	mh.param("wheelCenter",vehicle_angle_center,vehicle_angle_center);
	mh.param("gpsAngleCenter",gps_angle_center,gps_angle_center);
	mh.param("torqueMax",torque_max,torque_max);
	mh.param("xFrontAxelMid",x_middle_front,x_middle_front);
	mh.param("yFrontAxelMid",y_middle_front,y_middle_front);
	filter->mean_filter_init(&car_speed_, 5);
	filter->mean_filter_init(&eps_angle_, 5);
	filter->mean_filter_init(&target_output_, 10);
	memset(&resultcontrol, 0, sizeof(resultcontrol));	
	sc_loop_freq = 20;
}

void wsteercontrol::run()
{
	ros::Rate rate(sc_loop_freq);
	while (ros::ok())
	{	
		ros::spinOnce();
		StartDataProcess();
		rate.sleep();
	}
}

void wsteercontrol::SubMotionplanner(const ivpathplanner::ivmsgpath::ConstPtr &msg)
{
	motionpath = *msg;
}

void wsteercontrol::SubVAP(const ivmap::ivmapmsgvap::ConstPtr &msg)
{
	actuator_steerangle = msg->uisteerangle;
	actuator_speed = msg->v;
}

void wsteercontrol::StartDataProcess()
{
	amount_motion_path_point = motionpath.points.size();
	if (amount_motion_path_point > 3)
	{	
	
		CalculateMain();
	}
	else
	{
		resultcontrol.targetangle = 0;
		resultcontrol.torque = torque_max;
		resultcontrol.headingerror = 100;
		resultcontrol.lateralerror = 100;
		std::cout<<"Warnning : No road!!! "<<std::endl;
	}

	pub_.publish(resultcontrol);

}
  
void wsteercontrol::CalculateMain()
{
	//Get target road
	GetTargetPath();
	amount_target_path_point = target_path.x.size();
	path_end_flag = 0;
	double length_path_remain = target_path.distance(amount_target_path_point-1);
	
	if (length_path_remain < 1.0)
	{
		path_end_flag = 1;
	}
	
	//Get car position
	sCarCoord car_position = GetCarCoordinate();
	double angle_center = vehicle_angle_center + gps_angle_center;
	
	//Get feedback of actuator
	double car_speed = filter->mean_filter_apply(&car_speed_, actuator_speed);
	double eps_deg = filter->mean_filter_apply(&eps_angle_, actuator_steerangle);
	double wheel_rad = eps2wheel(eps_deg-vehicle_angle_center);
	double wheel_speed = 0;
	if (fabs(wheel_rad) < M_PI)
	{
		wheel_speed = fabs(car_speed / cos(wheel_rad));
	}
	else
	{
		wheel_speed = 0;
	}
	 
	// Get the relative position of the car and the road
	double id_nearest_point = GetNearestPointNum(car_position.x_car[0],
		car_position.y_car[0]);

	sTrackError track_err = GetTrackError(id_nearest_point, car_position);
	double heading_err = track_err.heading_err * M_PI / 180;
	double lateral_err = track_err.lateral_err;
	resultcontrol.headingerror = heading_err;
	resultcontrol.lateralerror = lateral_err;
	
	double target_wheel_angle = 0;
	if (1 == path_end_flag)
	{
		double heading_sum = 0;
		for (int i=0; i<amount_target_path_point; ++i)
		{
			heading_sum += target_path.angle(i) * M_PI/ 180;
		}
		target_wheel_angle = heading_sum / amount_target_path_point;
std::cout<<"target44444444444444444444444444444444 = "<<target_wheel_angle<<std::endl;
	}
	else
	{
		double vec_car2path = GetLineDirection(car_position.x_car[0], car_position.y_car[0],
		target_path.x(id_nearest_point), target_path.y(id_nearest_point));
		if (wheel_speed < 0.01)
		{
			target_wheel_angle = 0;
		}
		else if (fabs(lateral_err) > 5)
		{
			target_wheel_angle = 0;
		}
		else if (fabs(vec_car2path) < 60)
		{
			if (fabs(lateral_err) > 0.4)
			{
				target_wheel_angle = GetTargetAnglePureP(target_path.x(id_nearest_point), 
					target_path.y(id_nearest_point), car_position);
        std::cout<<"target1111111111111111111111111111111111111 = "<<target_wheel_angle<<std::endl;
			}
std::cout<<"target0000000000000000000000000000000000000 = "<<target_wheel_angle<<std::endl;
std::cout<<"vec_car2path = "<<vec_car2path<<std::endl;	
		}
		else
		{
			double k_coe = 0.3;
			target_wheel_angle = GetTargetAngleStanley(lateral_err, heading_err,
				wheel_speed, k_coe);	
      std::cout<<"target222222222222222222222222222222222222222 = "<<target_wheel_angle<<std::endl;
		}
			
	}
 std::cout<<"target3333333333333333333333333333333333333333 = "<<target_wheel_angle<<std::endl;

	double target_eps_angle1 = wheel2eps(target_wheel_angle);

	//limited by max eps angle
	double target_eps_angle2 = LimiteTargetAngle(target_eps_angle1);

	//meanfilter
	double target_eps_angle3 = filter->mean_filter_apply(&target_output_, target_eps_angle2);
	double target_eps_angle  = LimiteTargetAngle(target_eps_angle3);
	resultcontrol.targetangle = target_eps_angle + angle_center;
	resultcontrol.torque = torque_max;

}
double wsteercontrol::LimiteTargetAngle(double original_angle)
{
	if (original_angle > steerangle_max_r)
	{
		return (steerangle_max_r);
	}
	else if (original_angle < -steerangle_max_l)
	{
		return (-steerangle_max_l);
	}
	else 
	{
		return (original_angle);
	}
}

sCarCoord wsteercontrol::GetCarCoordinate ()
{
	sCarCoord car_coor;

    double mid_front_x  = x_middle_front;
	double mid_front_y  = y_middle_front;
	double left_rear_x  = x_middle_front - length_vehicle;
	double left_rear_y  = y_middle_front + width_vehicle*0.5;
	double right_rear_x = x_middle_front - length_vehicle;
	double right_rear_y = y_middle_front - width_vehicle*0.5;	
	double mid_rear_x   = (left_rear_x + right_rear_x)*0.5;
	double mid_rear_y   = (left_rear_y + right_rear_y)*0.5;	

	car_coor.x_car[0] = mid_front_x;
	car_coor.x_car[1] = left_rear_x;
	car_coor.x_car[2] = right_rear_x;
	car_coor.x_car[3] = mid_rear_x;
	car_coor.y_car[0] = mid_front_y;
	car_coor.y_car[1] = left_rear_y;
	car_coor.y_car[2] = right_rear_y;
	car_coor.y_car[3] = mid_rear_y;

	return car_coor;
}
double wsteercontrol::GetTargetAngleStanley(double lateral_error, 
	double heading_error, double wheel_speed, double k_coe)
{
	std::cout<<"lateral_error = "<<lateral_error<<std::endl;
	std::cout<<"heading_error = "<<heading_error<<std::endl;
	double head_part = heading_error;
	double late_part = 0;
	if (fabs(lateral_error) < DMIN_VALUE)
	{
		late_part = 0;
	}
	else if (wheel_speed*DMAX_VALUE < fabs(k_coe*lateral_error))
	{
		late_part = Signn(lateral_error) * M_PI * 0.5;
	} 
	else if (fabs(k_coe*lateral_error)*DMAX_VALUE < wheel_speed)
	{
		late_part = 0;
	} 
	else
	{
		late_part = atan2(k_coe*lateral_error, wheel_speed);
	}
std::cout<<"head_part  = "<<head_part<<std::endl;
std::cout<<"late_part  = "<<late_part<<std::endl;
	return (head_part + late_part);

}
double wsteercontrol::GetTargetAnglePureP(double reference_point_x, double reference_point_y,
	sCarCoord &vehicle_coordinate)
{
	double arc_radius = DMAX_VALUE;
	double arc_center_x = 0;
	double arc_center_y = 0;

	double vehicle_direction = GetLineDirection(vehicle_coordinate.x_car[3], 
		vehicle_coordinate.y_car[3], vehicle_coordinate.x_car[0], vehicle_coordinate.y_car[0]);
	double vehicle2reference_direction = GetLineDirection(vehicle_coordinate.x_car[3], 
		vehicle_coordinate.y_car[3], reference_point_x, reference_point_y);

	if (NearEqual(vehicle_direction, vehicle2reference_direction))
	{
		return 0;
	}
	else 
	{
		arc_center_x = vehicle_coordinate.x_car[3];
		arc_center_y = (pow((vehicle_coordinate.x_car[3]-reference_point_x), 2) 
			+ pow(reference_point_y, 2) + pow(vehicle_coordinate.y_car[3],2)) 
				/ (2*reference_point_y - 2*vehicle_coordinate.y_car[3]);

		arc_radius = GetLength(arc_center_x, arc_center_y, vehicle_coordinate.x_car[3],
			vehicle_coordinate.y_car[3]);
		double  center2wheel_l = GetLength(arc_center_x, arc_center_y, vehicle_coordinate.x_car[1],
			vehicle_coordinate.y_car[1]);
		double  center2wheel_r = GetLength(arc_center_x, arc_center_y, vehicle_coordinate.x_car[2],
			vehicle_coordinate.y_car[2]);

		if (center2wheel_l > center2wheel_r)
		{
			arc_radius = -arc_radius;
		}
		double wheel_base_length = GetLength(vehicle_coordinate.x_car[3], vehicle_coordinate.y_car[3],
			vehicle_coordinate.x_car[0], vehicle_coordinate.y_car[0]);

		return (atan(wheel_base_length/arc_radius));
	}
}
void wsteercontrol::GetTargetPath()
{
	target_path = GetOriginalPath();
	target_path = GetSmoothPath(target_path);
	AddDistanceToPath(target_path);
}

sPath wsteercontrol::GetOriginalPath() const
{
	sPath original_path;
	int amount_original_path_point = amount_motion_path_point;
	original_path.x        = VectorXd(amount_original_path_point);
	original_path.y        = VectorXd(amount_original_path_point);
	original_path.angle    = VectorXd(amount_original_path_point);
	original_path.distance = VectorXd(amount_original_path_point);

	for (int i=0; i<amount_original_path_point; ++i)
	{
		original_path.x(i)        = motionpath.points[i].x;
		original_path.y(i)        = motionpath.points[i].y;
		original_path.angle(i)    = motionpath.points[i].angle;
		original_path.distance(i) = 0;
	}
	return original_path;
}

sPath wsteercontrol::GetSmoothPath(sPath &original_path) const
{
	sPath smooth_path;
	int amount_original_path_point = original_path.x.size();
	int smooth_window = 8;

	if (amount_original_path_point <= smooth_window)
	{
		return original_path;
	}

	smooth_path.x         = VectorXd(amount_original_path_point);
	smooth_path.y         = VectorXd(amount_original_path_point);
	smooth_path.angle     = VectorXd(amount_original_path_point);
	smooth_path.distance  = VectorXd(amount_original_path_point);

	double xpointsum = 0;
	double ypointsum = 0;
	double roaddirectionmin = 0;
	double roaddirectionmax = 0;
	double roaddirectionsum = 0;
	for (int ismooth=0; ismooth<amount_original_path_point; ++ismooth)
	{
		if (ismooth <= smooth_window)
		{
			xpointsum = 0;
			ypointsum = 0;
			for (int i=0; i<=2*ismooth; ++i)
			{
				xpointsum = xpointsum + original_path.x(i);
				ypointsum = ypointsum + original_path.y(i);
			}
			smooth_path.x(ismooth) = xpointsum / (2*ismooth+1);
			smooth_path.y(ismooth) = ypointsum / (2*ismooth+1);
			smooth_path.angle(ismooth) = original_path.angle(ismooth);
		}

		else if (ismooth >= amount_original_path_point - smooth_window)
		{
			smooth_path.x(ismooth) = original_path.x(ismooth);
			smooth_path.y(ismooth) = original_path.y(ismooth);
			smooth_path.angle(ismooth) = original_path.angle(ismooth);			
		}

		else
		{
			xpointsum = 0;
			ypointsum = 0;
			for (int i = 0; i <= 2 * smooth_window; ++i)
			{
				xpointsum = xpointsum + original_path.x(ismooth+smooth_window-i);
				ypointsum = ypointsum + original_path.y(ismooth+smooth_window-i);
			}
			smooth_path.x(ismooth) = xpointsum / (2*smooth_window+1);
			smooth_path.y(ismooth) = ypointsum / (2*smooth_window+1);

			roaddirectionmin = original_path.angle(ismooth);
			roaddirectionmax = original_path.angle(ismooth);
			for (int i = 0; i <= 2 * smooth_window; ++i)
			{
				if (original_path.angle[ismooth+smooth_window-i] < roaddirectionmin)
				{
					roaddirectionmin = original_path.angle[ismooth+smooth_window-i];
				}
				if (original_path.angle[ismooth+smooth_window-i] > roaddirectionmax)
				{
					roaddirectionmax = original_path.angle[ismooth+smooth_window-i];
				}
			}

			if ((roaddirectionmax - roaddirectionmin) > 180)
			{
				smooth_path.angle(ismooth) = original_path.angle(ismooth);
			}
			else
			{
				roaddirectionsum = 0;
				for (int i=0; i<=2*smooth_window; ++i)
				{
					roaddirectionsum = roaddirectionsum + original_path.angle(ismooth+smooth_window-i);
				}
				smooth_path.angle(ismooth) = roaddirectionsum / (2*smooth_window+1);
			}
		}
		smooth_path.distance(ismooth) = 0;
	}
	
	return smooth_path;	
}
void wsteercontrol::AddDistanceToPath(sPath &smooth_path) 
{
	int amount_smooth_path_point = smooth_path.x.size();
	double dist_sum = 0;
	smooth_path.distance(0) = 0;
	for (int i=1; i<amount_smooth_path_point; ++i)
	{
		dist_sum += GetLength(smooth_path.x(i-1), smooth_path.y(i-1), smooth_path.x(i), smooth_path.y(i));
		smooth_path.distance(i) = dist_sum;
	}
}
int wsteercontrol::Signn(double value)
{
    int value_sign = 1;
    if (value >= 0)
    {
        value_sign = 1;
    }
    else
    {
        value_sign = -1;
    }
    return value_sign;
}

int wsteercontrol::GetNearestPointNum(double x_point, double y_point)
{
	double distance_to_car_min = 1000;
	double distance_point_to_vehicle = 100;
	int id_nearest_point = 1;
	for (int i=0; i<amount_target_path_point; i++)
	{
		distance_point_to_vehicle = sqrt(pow((target_path.x(i) - x_point),2) + 
										 pow((target_path.y(i) - y_point),2));
		if (distance_point_to_vehicle > distance_to_car_min + 5)
		{
			break;
		}
		if (distance_point_to_vehicle < distance_to_car_min)
		{
			distance_to_car_min = distance_point_to_vehicle;
			id_nearest_point = i;
		}
	}
	return id_nearest_point;
}

sTrackError wsteercontrol::GetTrackError(int id_nearest_point, sCarCoord &car_position)
{
	double lateral_err = 0;
	double heading_err = 0; 
	double lateral_dis = 0;

	double xfind = target_path.x(id_nearest_point);
	double yfind = target_path.y(id_nearest_point);
	sTrackError track_error;
	if (0 == id_nearest_point)
	{
		lateral_err = GetDist2Seg(car_position.x_car[0], car_position.y_car[0], 
								  target_path.x(0), target_path.y(0), 
								  target_path.x(1), target_path.y(1));
		heading_err = target_path.angle(0);
	}
	else if (amount_target_path_point-1 == id_nearest_point)
	{
		lateral_err = GetDist2Seg(car_position.x_car[0], car_position.y_car[0], 
							      target_path.x(id_nearest_point), 
							      target_path.y(id_nearest_point), 
							      target_path.x(id_nearest_point-1), 
							      target_path.y(id_nearest_point-1));
		heading_err = target_path.angle(amount_target_path_point-1);
	} 
	else
	{
		lateral_dis = GetDist2Seg(car_position.x_car[0], car_position.y_car[0], 
							      target_path.x(id_nearest_point), 
							      target_path.y(id_nearest_point), 
							      target_path.x(id_nearest_point-1), 
							      target_path.y(id_nearest_point-1));

		lateral_err = lateral_dis;
		heading_err = target_path.angle(id_nearest_point);

		lateral_dis = GetDist2Seg(car_position.x_car[0], car_position.y_car[0], 
							      target_path.x(id_nearest_point), 
							      target_path.y(id_nearest_point), 
							      target_path.x(id_nearest_point+1), 
							      target_path.y(id_nearest_point+1));	
		if (lateral_dis < lateral_err)
		{
			lateral_err = lateral_dis;
			heading_err = target_path.angle(id_nearest_point+1);
		}				 			   	
	}
	double left_err =  GetLength(car_position.x_car[1], car_position.y_car[1], 
								 target_path.x(id_nearest_point), 
								 target_path.y(id_nearest_point));
	double right_err = GetLength(car_position.x_car[2], car_position.y_car[2], 
		                         target_path.x(id_nearest_point), 
		                         target_path.y(id_nearest_point));

	if (left_err > right_err)
	{
		lateral_err = -lateral_err;
	}

	track_error.lateral_err = lateral_err;
	track_error.heading_err = heading_err;

	return track_error;
}


double wsteercontrol::wheel2eps(double wheel_angle)
{	
	if (wheel_angle < 0)
	{
		return (-1 * wheel_angle * k_ratio_r * 180.0 / M_PI);
	}
	else
	{
		return (-1 * wheel_angle * k_ratio_l * 180.0 / M_PI);
	}
}

double wsteercontrol::eps2wheel(double eps_angle)
{
	double wheel_rad = 0;
	if (eps_angle > 0)
	{
		wheel_rad = - (eps_angle-vehicle_angle_center) / k_ratio_r * M_PI / 180;		
	}
	else
	{
		wheel_rad = - (eps_angle-vehicle_angle_center) / k_ratio_l * M_PI / 180;		
	}
	return wheel_rad;
}

double wsteercontrol::GetLength(double x_point1, double y_point1, double x_point2, double y_point2)
{
	return (sqrt(pow(x_point1 - x_point2, 2) + pow(y_point1 - y_point2, 2)));
}

double wsteercontrol::GetDist2Seg(double x, double y, double x1, double y1, double x2, double y2)
{
	double cross = (x2 - x1) * (x - x1) + (y2 - y1) * (y - y1);
	if (cross <= 0) 
	{
		return (GetLength(x, y, x1, y1));
	}	

	double d2 = GetLength(x1, y1, x2, y2);
	if (cross >= d2 * d2) 
	{
		return (GetLength(x, y, x2, y2));
	}
	
	double r = cross / d2;
	double px = x1 + (x2 - x1) * r;
	double py = y1 + (y2 - y1) * r;
	double dist2seg = GetLength(x, y, px, py);
	return dist2seg;
}

double wsteercontrol::GetLineDirection(double x_from, double y_from, double x_to, double y_to)  //-180~180
{
	double vector_x = x_to - x_from;
	double vector_y = y_to - y_from;
	double vector_norm = sqrt(pow(vector_x, 2) + pow(vector_y, 2));
	double cosvalue = vector_x / vector_norm;
	double line_direction = acos (cosvalue) * 180 / M_PI;

	int x_equal = NearEqual(x_from, x_to);
	int y_equal = NearEqual(y_from, y_to); 
	if (x_equal && y_equal)
	{
		return 0;
	}
    else if (vector_y < 0)
    {
        return (-line_direction);
    }
	else
	{
		return (line_direction);
	}
}

int wsteercontrol::NearEqual(double num1, double num2)
{
	if ((num1-num2) > -DMIN_VALUE && (num1-num2) < DMIN_VALUE)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
