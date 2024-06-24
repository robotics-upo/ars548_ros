/**
 * @file ars548_driver_filter.cpp
 * @brief This is an example of usage of the driver. In this case we subscribe to the object message that the driver sends us and we filter the data by its velocity.
*/

#include <ars548_driver/ars548_filter_example.hpp>

int main(int argc,char* argv[]){
   rclcpp::init(argc,argv);
   rclcpp::spin(std::make_shared<ARS548FilterExample>());
   rclcpp::shutdown();
   return 0;
}
