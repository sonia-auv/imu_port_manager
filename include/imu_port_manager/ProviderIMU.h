#pragma once

#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_srvs/srv/empty.hpp>
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
            

            bool tare();
            bool reset();
            bool factory_reset();
            bool magnetic_disturbance(std_srvs::srv::SetBool::Request &rsq);
            bool acceleration_disturbance(std_srvs::srv::SetBool::Request &rsq);
            bool velocity_compensation(std_srvs::srv::SetBool::Request &rsq);
            bool asyn_output_pause(std_srvs::srv::SetBool::Request &rsq);

            void dvl_velocity(const geometry_msgs::msg::Twist::SharedPtr& msg);
            void asyn_Data_frequency_callback(const std_msgs::msg::UInt8::SharedPtr& msg);
            void vpe_basic_control_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr& msg);
            void magnetometer_calibration_control_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr& msg);
            void delta_theta_delta_velocity_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr& msg);
            void imu_filtering_configuration_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr& msg);

            void reader();
            //void send_register(Register& reg);
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
    };
}