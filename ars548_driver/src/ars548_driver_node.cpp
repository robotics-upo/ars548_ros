/**
 * @file ars548_driver_node.cpp 
 */
#include "ros/ros.h"
#include "ars548_driver/ars548_driver.hpp"

int main(int argc,char* argv[]){
  ros::init(argc,argv, "ars548_driver_node");

  ARS548Driver driver;

  ros::Rate loop_rate(100);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}


