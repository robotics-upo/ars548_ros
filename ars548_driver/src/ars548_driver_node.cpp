/**
 * @file ars548_driverNode.cpp 
 */
#include "rclcpp/rclcpp.hpp"
#include "ars548_driver/ars548_driver.hpp"

int main(int argc,char* argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ars548_driver>());
    rclcpp::shutdown();
    return 0;
}


