#include "imu_port_manager/ImuProvider.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto imu= std::make_shared<imu_provider::ImuProvider>();
    
    if (!imu->OpenPort())
    {   
        std::cout << "Could not open port..." << std::endl;
        return EXIT_FAILURE;
    }
    rclcpp::spin(imu);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
