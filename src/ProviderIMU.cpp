#include <sstream>
#include "boost/log/trivial.hpp"
#include "imu_port_manager/ProviderIMU.h"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

namespace provider_imu
{
    ProviderIMU::ProviderIMU()
        : Node("provider_imu"), _rs485Connection("/dev/IMU", B115200, true)
    {
        // Publisher
        publisher = this->create_publisher<sensor_msgs::msg::Imu>("provider_imu/imu_info", 100);

        // Subscribers
        dvl_subscriber = this->create_subscription<geometry_msgs::msg::Twist>("/proc_nav/dvl_velocity", 100, std::bind(&ProviderIMU::dvl_velocity, this, _1));
        vpe_basic_control = this->create_subscription<std_msgs::msg::UInt8MultiArray>("/provider_imu/vpe_basic_control", 10, std::bind(&ProviderIMU::vpe_basic_control_callback, this, _1));
        magnetometer_calibration_control = this->create_subscription<std_msgs::msg::UInt8MultiArray>("/provider_imu/magnetometer_calibration_control", 10, std::bind(&ProviderIMU::magnetometer_calibration_control_callback, this, _1));
        delta_theta_delta_velocity = this->create_subscription<std_msgs::msg::UInt8MultiArray>("/provider_imu/delta_theta_delta_velocity", 10, std::bind(&ProviderIMU::delta_theta_delta_velocity_callback, this, _1));
        imu_filtering_configuration = this->create_subscription<std_msgs::msg::UInt8MultiArray>("/provider_imu/imu_filtering_configuration", 10, std::bind(&ProviderIMU::imu_filtering_configuration_callback, this, _1));

        // Services
        tare_srv = this->create_service<std_srvs::srv::Trigger>("/provider_imu/tare", std::bind(&ProviderIMU::tare, this, _1, _2));
        reset_srv = this->create_service<std_srvs::srv::Trigger>("/provider_imu/reset", std::bind(&ProviderIMU::reset, this, _1, _2));
        factory_reset_srv = this->create_service<std_srvs::srv::Trigger>("/provider_imu/factory_reset", std::bind(&ProviderIMU::factory_reset, this, _1, _2));
        magnetic_disturbance_srv = this->create_service<std_srvs::srv::SetBool>("/provider_imu/magnetic_disturbance", std::bind(&ProviderIMU::magnetic_disturbance, this, _1, _2));
        acceleration_disturbance_srv = this->create_service<std_srvs::srv::SetBool>("/provider_imu/acceleration_disturbance", std::bind(&ProviderIMU::acceleration_disturbance, this, _1, _2));
        velocity_compensation_srv = this->create_service<std_srvs::srv::SetBool>("/provider_imu/velocity_compensation", std::bind(&ProviderIMU::velocity_compensation, this, _1, _2));
        asyn_output_pause_srv  = this->create_service<std_srvs::srv::SetBool>("/provider_imu/pause", std::bind(&ProviderIMU::asyn_output_pause , this, _1, _2));

        _reg_15.thread = std::thread(std::bind(&ProviderIMU::send_register_15, this));
        _reg_239.thread = std::thread(std::bind(&ProviderIMU::send_register_239, this));
        _reg_240.thread = std::thread(std::bind(&ProviderIMU::send_register_240, this));
        read_thread = std::thread(std::bind(&ProviderIMU::reader, this));
        _err.thread = std::thread(std::bind(&ProviderIMU::send_err, this));
    }

    ProviderIMU::~ProviderIMU()
    {
        _reg_15.stop_thread = true;
        _reg_239.stop_thread = true;
        _reg_240.stop_thread = true;
        _err.stop_thread = true;
        _reader_stop_thread = true;
    }

    bool ProviderIMU::OpenPort()
    {
        bool res = _rs485Connection.OpenPort();
        if (res)
        {
            _rs485Connection.Flush();
        }
        return res;
    }

    uint8_t ProviderIMU::calculeCheckSum(const std::string data)
    {
        uint8_t check = 0;

        for(unsigned int i = 1; i < data.size(); i++)
            check ^= data[i];
        
        return check;
    }
    
    void ProviderIMU::appendCheckSum(std::string &data)
    {
        std::stringstream ss;
        uint8_t checksum = calculeCheckSum(data);
        ss << data << std::string("*") << std::hex << checksum;
        data = ss.str();
    }

    bool ProviderIMU::confirmCheckSum(std::string &data)
    {
        try
        {
            std::string checksumData = data.substr(0, data.find("*", 0));
            uint8_t calculatedChecksum = calculeCheckSum(checksumData);
            uint8_t originalChecksum = std::stoi(data.substr(data.find("*", 0)+1, 2), nullptr, 16);
            return originalChecksum == calculatedChecksum;
        }
        catch(...)
        {
            BOOST_LOG_TRIVIAL(info)<<"IMU : Bad packet checksum";
            return false;
        }
    }

    void ProviderIMU::tare(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        _rs485Connection.Transmit("$VNTAR*5F\n");
        std::this_thread::sleep_for(0.1s);
        response->success = true;
    }

    void ProviderIMU::reset(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        _rs485Connection.Transmit("$VNRST*4D\n");
        std::this_thread::sleep_for(0.1s);
        response->success = true;
    }

    void ProviderIMU::factory_reset(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        _rs485Connection.Transmit("$VNRFS*5F\n");
        std::this_thread::sleep_for(0.1s);
        response->success = true;
    }

    void ProviderIMU::magnetic_disturbance(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        std::stringstream ss;

        _writerMutex.lock();
        ss << "$VNKMD," << std::to_string((uint8_t)request->data);
        std::string send_data = ss.str();
        appendCheckSum(send_data);

        _rs485Connection.Transmit(send_data);
        std::this_thread::sleep_for(0.1s);

        _writerMutex.unlock();
        response->success = true;
    }

    void ProviderIMU::acceleration_disturbance(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        std::stringstream ss;

        _writerMutex.lock();
        ss << "$VNKAD," << std::to_string((uint8_t)request->data);
        std::string send_data = ss.str();
        appendCheckSum(send_data);

        _rs485Connection.Transmit(send_data);
        std::this_thread::sleep_for(0.1s);

        _writerMutex.unlock();
        response->success = true;
    }

    void ProviderIMU::velocity_compensation(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        std::stringstream ss;

        _writerMutex.lock();
        ss << "$VNWNV,51" << std::to_string((uint8_t)request->data) << ",0.1,0.01";
        std::string send_data = ss.str();
        appendCheckSum(send_data);

        _rs485Connection.Transmit(send_data);
        std::this_thread::sleep_for(0.1s);

        _writerMutex.unlock();
        response->success = true;
    }

    void ProviderIMU::asyn_output_pause(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        std::stringstream ss;

        _writerMutex.lock();
        ss << "$VNASY," << std::to_string((uint8_t)request->data);
        std::string send_data = ss.str();
        appendCheckSum(send_data);

        _rs485Connection.Transmit(send_data);
        std::this_thread::sleep_for(0.1s);

        _writerMutex.unlock();
        response->success = true;
    }

    void ProviderIMU::dvl_velocity(const std::shared_ptr<geometry_msgs::msg::Twist> msg)
    {
        std::stringstream ss;

        std::unique_lock<std::mutex> mlock(_writerMutex);
        ss << "$VNWRG,50," << std::to_string((float)msg->linear.x) << "," << std::to_string((float)msg->linear.y) << "," << std::to_string((float)msg->linear.z);
        std::string send_data = ss.str();
        appendCheckSum(send_data);

        _rs485Connection.Transmit(send_data);
    }

    /*void ProviderIMU::asyn_Data_frequency_callback(const std_msgs::msg::UInt8::SharedPtr &msg)
    {
        std::stringstream ss;

        _writerMutex.lock();

        ss << "$VNWNV,07," << std::to_string(msg->data);
        std::string send_data = ss.str();
        appendCheckSum(send_data);

        _rs485Connection.Transmit(send_data);
        std::this_thread::sleep_for(0.5s);

        _writerMutex.unlock();
    }*/

    void ProviderIMU::vpe_basic_control_callback(const std::shared_ptr<std_msgs::msg::UInt8MultiArray> msg)
    {
        std::stringstream ss;

        _writerMutex.lock();

        ss << "$VNWNV,35," << std::to_string(msg->data.at(0)) << "," << std::to_string(msg->data.at(1))
           << "," << std::to_string(msg->data.at(2)) << "," << std::to_string(msg->data.at(3));
        std::string send_data = ss.str();
        appendCheckSum(send_data);

        _rs485Connection.Transmit(send_data);
        std::this_thread::sleep_for(0.5s);

        _writerMutex.unlock();
    }

    void ProviderIMU::magnetometer_calibration_control_callback(const std::shared_ptr<std_msgs::msg::UInt8MultiArray> msg)
    {
        std::stringstream ss;

        _writerMutex.lock();

        ss << "$VNWNV,35," << std::to_string(msg->data.at(0)) << "," << std::to_string(msg->data.at(1))
           << "," << std::to_string(msg->data.at(2)) << "," << std::to_string(msg->data.at(3));
        std::string send_data = ss.str();
        appendCheckSum(send_data);

        _rs485Connection.Transmit(send_data);
        std::this_thread::sleep_for(0.5s);

        _writerMutex.unlock();
    }

    void ProviderIMU::delta_theta_delta_velocity_callback(const std::shared_ptr<std_msgs::msg::UInt8MultiArray> msg)
    {
        std::stringstream ss;

        _writerMutex.lock();
        
        ss << "$VNWNV,82," << std::to_string(msg->data.at(0)) << "," << std::to_string(msg->data.at(1))
           << "," << std::to_string(msg->data.at(2)) << "," << std::to_string(msg->data.at(3));
        std::string send_data = ss.str();
        appendCheckSum(send_data);

        _rs485Connection.Transmit(send_data);
        std::this_thread::sleep_for(0.5s);

        _writerMutex.unlock();
    }

    void ProviderIMU::imu_filtering_configuration_callback(const std::shared_ptr<std_msgs::msg::UInt8MultiArray> msg)
    {
        std::stringstream ss;

        _writerMutex.lock();

        ss << "$VNWNV,85," << std::to_string(msg->data.at(0)) << "," << std::to_string(msg->data.at(1)) << "," << std::to_string(msg->data.at(2))
           << "," << std::to_string(msg->data.at(4)) << "," << std::to_string(msg->data.at(5)) << "," << std::to_string(msg->data.at(6))
           << "," << std::to_string(msg->data.at(7)) << "," << std::to_string(msg->data.at(8)) << "," << std::to_string(msg->data.at(9));
        std::string send_data = ss.str();
        appendCheckSum(send_data);

        _rs485Connection.Transmit(send_data);
        std::this_thread::sleep_for(0.5s);

        _writerMutex.unlock();
    }

    void ProviderIMU::reader()
    {
        char buffer[BUFFER_SIZE];

        while (!_reader_stop_thread)
        {
            do
            {   
                _rs485Connection.ReadOnce((uint8_t*)buffer,0);
            } while (buffer[0] != '$');

            int i;

            for (i = 1; buffer[i - 1] != '\n' && i < BUFFER_SIZE; i++)
            {
                _rs485Connection.ReadOnce((uint8_t *)buffer, i);
            }

            if (i >= BUFFER_SIZE)
            {
                continue;
            }

            buffer[i] = 0;

            if (!strncmp(&buffer[3], REG_15, 3))
            {
                std::unique_lock<std::mutex> mlock(_reg_15.mutex);
                _reg_15.str = std::string(buffer);
                _reg_15.cond.notify_one();
            }
            else if (!strncmp(&buffer[3], REG_239, 3))
            {
                std::unique_lock<std::mutex> mlock(_reg_239.mutex);
                _reg_239.str = std::string(buffer);
                _reg_239.cond.notify_one();
            }
            else if (!strncmp(&buffer[3], REG_240, 3))
            {
                std::unique_lock<std::mutex> mlock(_reg_240.mutex);
                _reg_240.str = std::string(buffer);
                _reg_240.cond.notify_one();
            }
            else if (!strncmp(&buffer[3], ERR_STR, 3))
            {
                std::unique_lock<std::mutex> mlock(_err.mutex);
                _err.str = std::string(buffer);
                _err.cond.notify_one();
            }
            else
            {
                BOOST_LOG_TRIVIAL(info)<<buffer;
            }

        } // end while
    }     // end reader

    void ProviderIMU::send_err()
    {
        while (!_err.stop_thread)
        {
            sensor_msgs::msg::Imu msg;
            std::string parameter = "";

            std::unique_lock<std::mutex> mlock(_err.mutex);
            _err.cond.wait(mlock);

            try
            {
                if ((!_err.str.empty()) && confirmCheckSum(_err.str))
                {
                    std::stringstream ss(_err.str);

                    std::getline(ss, parameter, ',');

                    std::getline(ss, parameter, ',');

                    BOOST_LOG_TRIVIAL(info)<<"Error : "<<parameter.c_str();
                }
            }
            catch(...)
            {
                BOOST_LOG_TRIVIAL(info)<<"IMU : Bad packet error";
            }
            
            
        }
    }
    
    void ProviderIMU::send_register_15()
    {
        while (!_reg_15.stop_thread)
        {
            sensor_msgs::msg::Imu msg;
            std::string parameter = "";

            std::unique_lock<std::mutex> mlock(_reg_15.mutex);
            _reg_15.cond.wait(mlock);

            try
            {
                if ((!_reg_15.str.empty()) && confirmCheckSum(_reg_15.str))
                {
                    std::stringstream ss(_reg_15.str);

                    std::getline(ss, parameter, ',');

                    std::getline(ss, parameter, ',');
                    msg.orientation.x = std::stof(parameter);

                    std::getline(ss, parameter, ',');
                    msg.orientation.y = std::stof(parameter);

                    std::getline(ss, parameter, ',');
                    msg.orientation.z = std::stof(parameter);

                    std::getline(ss, parameter, ',');
                    msg.linear_acceleration.x = std::stof(parameter);

                    std::getline(ss, parameter, ',');
                    msg.linear_acceleration.y = std::stof(parameter);

                    std::getline(ss, parameter, ',');
                    msg.linear_acceleration.z = std::stof(parameter);

                    std::getline(ss, parameter, ',');
                    msg.angular_velocity.x = std::stof(parameter);

                    std::getline(ss, parameter, ',');
                    msg.angular_velocity.y = std::stof(parameter);

                    std::getline(ss, parameter, '*');
                    msg.angular_velocity.z = std::stof(parameter);
                    publisher->publish(msg);
                }
            }
            catch(...)
            {
                BOOST_LOG_TRIVIAL(info)<<"IMU : Bad packet register 15";
            }   
        }
    }

    void ProviderIMU::send_register_239()
    {
        while (!_reg_239.stop_thread)
        {
            sensor_msgs::msg::Imu msg;
            std::string parameter = "";

            std::unique_lock<std::mutex> mlock(_reg_239.mutex);
            _reg_239.cond.wait(mlock);

            try
            {
                if ((!_reg_239.str.empty()) && confirmCheckSum(_reg_239.str))
                {
                    std::stringstream ss(_reg_239.str);

                    std::getline(ss, parameter, ',');

                    std::getline(ss, parameter, ',');
                    msg.orientation.x = std::stof(parameter);

                    std::getline(ss, parameter, ',');
                    msg.orientation.y = std::stof(parameter);

                    std::getline(ss, parameter, ',');
                    msg.orientation.z = std::stof(parameter);

                    std::getline(ss, parameter, ',');
                    msg.linear_acceleration.x = std::stof(parameter);

                    std::getline(ss, parameter, ',');
                    msg.linear_acceleration.y = std::stof(parameter);

                    std::getline(ss, parameter, ',');
                    msg.linear_acceleration.z = std::stof(parameter);

                    std::getline(ss, parameter, ',');
                    msg.angular_velocity.x = std::stof(parameter);

                    std::getline(ss, parameter, ',');
                    msg.angular_velocity.y = std::stof(parameter);

                    std::getline(ss, parameter, '*');
                    msg.angular_velocity.z = std::stof(parameter);
                    publisher->publish(msg);
                }
            }
            catch(...)
            {
                BOOST_LOG_TRIVIAL(info)<<"IMU : Bad packet register 239";
            }
        }
    }

    void ProviderIMU::send_register_240()
    {
        while (!_reg_240.stop_thread)
        {
            sensor_msgs::msg::Imu msg;
            std::string parameter = "";

            std::unique_lock<std::mutex> mlock(_reg_240.mutex);
            _reg_240.cond.wait(mlock);

            try
            {
                if ((!_reg_240.str.empty()) && confirmCheckSum(_reg_240.str))
                {
                    std::stringstream ss(_reg_15.str);

                    std::getline(ss, parameter, ',');

                    std::getline(ss, parameter, ',');
                    msg.orientation.x = std::stof(parameter);

                    std::getline(ss, parameter, ',');
                    msg.orientation.y = std::stof(parameter);

                    std::getline(ss, parameter, ',');
                    msg.orientation.z = std::stof(parameter);

                    std::getline(ss, parameter, ',');
                    msg.linear_acceleration.x = std::stof(parameter);

                    std::getline(ss, parameter, ',');
                    msg.linear_acceleration.y = std::stof(parameter);

                    std::getline(ss, parameter, ',');
                    msg.linear_acceleration.z = std::stof(parameter);

                    std::getline(ss, parameter, ',');
                    msg.angular_velocity.x = std::stof(parameter);

                    std::getline(ss, parameter, ',');
                    msg.angular_velocity.y = std::stof(parameter);

                    std::getline(ss, parameter, '*');
                    msg.angular_velocity.z = std::stof(parameter);
                    publisher->publish(msg);
                }
            }
            catch(...)
            {
                BOOST_LOG_TRIVIAL(info)<<"IMU : Bad packet register 240";
            }
        }
    }
} // end namespace