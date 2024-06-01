#pragma once

#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <sonia_common_cpp/SerialConn.h>

#include <stdio.h>
#include <string>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "SharedQueue.h"

namespace provider_imu
{
    struct Register
    {
        std::string str="";
        std::mutex mutex;
        std::condition_variable cond;
        std::thread thread;
        bool stop_thread;             
    };

    class ProviderIMU:public rclcpp::Node
    {
        public:
            ProviderIMU();
            ~ProviderIMU();
            bool OpenPort();
        private:
            Register _reg_15; 
            Register _reg_239;
            Register _reg_240;
            Register _err;
            
            bool _reader_stop_thread=false;
            std::thread read_thread;

            std::mutex _writerMutex;

            uint8_t calculeCheckSum(std::string data);
            void appendCheckSum(std::string& data);
            bool confirmCheckSum(std::string& data);
            

            void tare(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
            void reset(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
            void factory_reset(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
            void magnetic_disturbance(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);
            void acceleration_disturbance(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);
            void velocity_compensation(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);
            void asyn_output_pause(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);

            void dvl_velocity(const std::shared_ptr<geometry_msgs::msg::Twist>  msg);
            //void asyn_Data_frequency_callback(const std_msgs::msg::UInt8::SharedPtr& msg);
            void vpe_basic_control_callback(const std::shared_ptr<std_msgs::msg::UInt8MultiArray> msg);
            void magnetometer_calibration_control_callback(const std::shared_ptr<std_msgs::msg::UInt8MultiArray> msg);
            void delta_theta_delta_velocity_callback(const std::shared_ptr<std_msgs::msg::UInt8MultiArray> msg);
            void imu_filtering_configuration_callback(const std::shared_ptr<std_msgs::msg::UInt8MultiArray> msg);

            void reader();
            void send_register_15();
            void send_register_239();
            void send_register_240();
            void send_err();

            static const int BUFFER_SIZE= 4096;
            const char* REG_15 = "QMR";
            const char* REG_239 = "YBA";
            const char* REG_240 = "YIA";
            const char* ERR_STR = "ERR";

            sonia_common_cpp::SerialConn _rs485Connection;

            rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher;

            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr dvl_subscriber;
            rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr vpe_basic_control;
            rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr magnetometer_calibration_control;
            rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr delta_theta_delta_velocity;
            rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr imu_filtering_configuration;

            rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr tare_srv;
            rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv;
            rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr factory_reset_srv;
            rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr magnetic_disturbance_srv;
            rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr acceleration_disturbance_srv;
            rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr velocity_compensation_srv;
            rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr asyn_output_pause_srv;
    };
}