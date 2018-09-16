#include "rclcpp/rclcpp.hpp"


int main(int argc, char *argv [])
{
  rclcpp::init(argc,arv);
  
  auto rtl_sdr = std::make_shared<rtl_sdr_driver::RTLSDRDriverNode>();
  rclcpp::spin(rtl_sdr);
  
  rclcpp::shutdown();
  return 0;
  
 }
