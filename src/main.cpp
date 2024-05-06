#include "imu_port_manager/ProviderIMU.h"
#include <stdlib.h>
#include <iostream>
#include <chrono>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto imu= std::make_shared<provider_imu::ProviderIMU>();
    
    //auto interface = std::make_shared<sonia_hw_interface::RS485Interface>();
    if (!imu->OpenPort())
    {   
        std::cout << "Could not open port..." << std::endl;
        return EXIT_FAILURE;
    }
    rclcpp::spin(imu);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
