/*******************************************************************************
* Copyright (C) 2015-2020, idriverplus(BeiJing ZhiXingZhe, Inc.)
* 
* NodeName: ivsteercontrol
* FileName: steercontrol.h, steercontrol.cpp
* 
* Description: 
  1. Get the target road from ivpathplanner
  2. Get the vehicle speed and steering angle from ivmap
* 3. calculate the steering angle to track the target path
* 4. calculate the torque
*
* History: 
* jianwei         17/06/20    1.0.0    build this module. 
********************************************************************************/
#include "wsteercontrol.h"
int main(int argc, char *argv[])
{
  ros::init(argc, argv,"ivsteercontrol");
  ros::NodeHandle mh;
  std::string avos("lsav");
  mh.param("avos", avos, avos);

  wsteercontrol node(mh);

  node.run();
  return 0;
}
