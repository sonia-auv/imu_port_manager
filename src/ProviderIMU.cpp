#include <sstream>
#include "imu_port_manager/ProviderIMU.h"

namespace provider_imu
{
    ProviderIMU::ProviderIMU()
        : Node("provider_imu"), _rs485Connection("/dev/IMU", B115200, true)
    {
        publisher = this->create_publisher<sensor_msgs::msg::Imu>("provider_imu/imu_info", 100);

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
        std::string checksumData = data.substr(0, data.find("*", 0));
        uint8_t calculatedChecksum = calculeCheckSum(checksumData);
        uint8_t originalChecksum = std::stoi(data.substr(data.find("*", 0)+1, 2), nullptr, 16);
        return originalChecksum == calculatedChecksum;
    }

    bool ProviderIMU::tare()
    {
        _rs485Connection.Transmit("$VNTAR*5F\n");
        return true;
    }

    bool ProviderIMU::reset()
    {
        _rs485Connection.Transmit("$VNRST*4D\n");
        return true;
    }

    bool ProviderIMU::factory_reset()
    {
        _rs485Connection.Transmit("$VNRFS*5F\n");
        return true;
    }

    bool ProviderIMU::magnetic_disturbance(std_srvs::srv::SetBool::Request &rsq)
    {
        std::stringstream ss;

        _writerMutex.lock();
        ss << "$VNKMD," << std::to_string((uint8_t)rsq.data);
        std::string send_data = ss.str();
        appendCheckSum(send_data);

        _rs485Connection.Transmit(send_data);
        _writerMutex.unlock();
        return true;
    }

    bool ProviderIMU::acceleration_disturbance(std_srvs::srv::SetBool::Request &rsq)
    {
        std::stringstream ss;

        _writerMutex.lock();
        ss << "$VNKAD," << std::to_string((uint8_t)rsq.data);
        std::string send_data = ss.str();
        appendCheckSum(send_data);

        _rs485Connection.Transmit(send_data);
        _writerMutex.unlock();
        return true;
    }

    bool ProviderIMU::velocity_compensation(std_srvs::srv::SetBool::Request &rsq)
    {
        std::stringstream ss;

        _writerMutex.lock();
        ss << "$VNWNV,51" << std::to_string((uint8_t)rsq.data) << ",0.1,0.01";
        std::string send_data = ss.str();
        appendCheckSum(send_data);

        _rs485Connection.Transmit(send_data);
        _writerMutex.unlock();
        return true;
    }

    bool ProviderIMU::asyn_output_pause(std_srvs::srv::SetBool::Request &rsq)
    {
        std::stringstream ss;

        _writerMutex.lock();
        ss << "$VNASY," << std::to_string((uint8_t)rsq.data);
        std::string send_data = ss.str();
        appendCheckSum(send_data);

        _rs485Connection.Transmit(send_data);
        _writerMutex.unlock();
        return true;
    }
    void ProviderIMU::dvl_velocity(const geometry_msgs::msg::Twist::SharedPtr &msg)
    {
        std::stringstream ss;

        std::unique_lock<std::mutex> mlock(_writerMutex);
        ss << "$VNWRG,50," << std::to_string((float)msg->linear.x) << "," << std::to_string((float)msg->linear.y) << "," << std::to_string((float)msg->linear.z);
        std::string send_data = ss.str();
        appendCheckSum(send_data);

        _rs485Connection.Transmit(send_data);
    }

    void ProviderIMU::asyn_Data_frequency_callback(const std_msgs::msg::UInt8::SharedPtr &msg)
    {
        std::stringstream ss;

        _writerMutex.lock();
        ss << "$VNWNV,07," << std::to_string(msg->data);
        std::string send_data = ss.str();
        appendCheckSum(send_data);

        _rs485Connection.Transmit(send_data);
        _writerMutex.unlock();
    }

    void ProviderIMU::vpe_basic_control_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr &msg)
    {
        std::stringstream ss;

        _writerMutex.lock();
        ss << "$VNWNV,35," << std::to_string(msg->data.at(0)) << "," << std::to_string(msg->data.at(1))
           << "," << std::to_string(msg->data.at(2)) << "," << std::to_string(msg->data.at(3));
        std::string send_data = ss.str();
        appendCheckSum(send_data);

        _rs485Connection.Transmit(send_data);
        _writerMutex.unlock();
    }

    void ProviderIMU::magnetometer_calibration_control_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr &msg)
    {
        std::stringstream ss;

        _writerMutex.lock();
        ss << "$VNWNV,35," << std::to_string(msg->data.at(0)) << "," << std::to_string(msg->data.at(1))
           << "," << std::to_string(msg->data.at(2)) << "," << std::to_string(msg->data.at(3));
        std::string send_data = ss.str();
        appendCheckSum(send_data);

        _rs485Connection.Transmit(send_data);
        _writerMutex.unlock();
    }

    void ProviderIMU::delta_theta_delta_velocity_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr &msg)
    {
        std::stringstream ss;

        _writerMutex.lock();
        ss << "$VNWNV,82," << std::to_string(msg->data.at(0)) << "," << std::to_string(msg->data.at(1))
           << "," << std::to_string(msg->data.at(2)) << "," << std::to_string(msg->data.at(3));
        std::string send_data = ss.str();
        appendCheckSum(send_data);

        _rs485Connection.Transmit(send_data);
        _writerMutex.unlock();
    }
    void ProviderIMU::imu_filtering_configuration_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr &msg)
    {
        std::stringstream ss;

        _writerMutex.lock();
        ss << "$VNWNV,85," << std::to_string(msg->data.at(0)) << "," << std::to_string(msg->data.at(1)) << "," << std::to_string(msg->data.at(2))
           << "," << std::to_string(msg->data.at(4)) << "," << std::to_string(msg->data.at(5)) << "," << std::to_string(msg->data.at(6))
           << "," << std::to_string(msg->data.at(7)) << "," << std::to_string(msg->data.at(8)) << "," << std::to_string(msg->data.at(9));
        std::string send_data = ss.str();
        appendCheckSum(send_data);

        _rs485Connection.Transmit(send_data);
        _writerMutex.unlock();
    }

    void ProviderIMU::reader()
    {
        char buffer[BUFFER_SIZE];

        while (!_reader_stop_thread)
        {
            do
            {   
                _rs485Connection.ReadPackets(1,(uint8_t*)buffer);
                std::cout<<buffer[0]<<std::endl;
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
            std::cout<<buffer[3]<<std::endl;

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
            if ((!_err.str.empty()) && confirmCheckSum(_err.str))
            {
                std::stringstream ss(_err.str);

                std::getline(ss, parameter, ',');

                std::getline(ss, parameter, ',');
            }
        }
    }
    /*void ProviderIMU::send_register(Register& reg)
    {
        while(!reg.stop_thread)
        {
            sensor_msgs::msg::Imu msg;
            std::string parameter ="";

            std::unique_lock<std::mutex> mlock(reg.mutex);
            reg.cond.wait(mlock);
            if((!reg.str.empty()) && confirmCheckSum(reg.str))
                {
                    std::stringstream ss(reg.str);

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

    }*/
    void ProviderIMU::send_register_15()
    {
        while (!_reg_15.stop_thread)
        {
            sensor_msgs::msg::Imu msg;
            std::string parameter = "";

            std::cout<<"Hello"<<std::endl;

            std::unique_lock<std::mutex> mlock(_reg_15.mutex);
            _reg_15.cond.wait(mlock);
            std::cout<<"Hello2"<<std::endl;
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
    }
    void ProviderIMU::send_register_239()
    {
        while (!_reg_239.stop_thread)
        {
            sensor_msgs::msg::Imu msg;
            std::string parameter = "";

            std::unique_lock<std::mutex> mlock(_reg_239.mutex);
            _reg_239.cond.wait(mlock);
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
    }
    void ProviderIMU::send_register_240()
    {
        while (!_reg_240.stop_thread)
        {
            sensor_msgs::msg::Imu msg;
            std::string parameter = "";

            std::unique_lock<std::mutex> mlock(_reg_240.mutex);
            _reg_240.cond.wait(mlock);
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
    }
} // end namespace