//
// Created by charles on 23/06/24.
//

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace multivesc_driver
{
    using namespace std::chrono_literals;


    //! VESC motor control node

    class VescMotor : public rclcpp::Node
    {
    public:
        VescMotor(std::string name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
          : Node(name, options)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "VescMotor created");
            this->declare_parameter<int64_t>("id");
            this->declare_parameter<std::string>("control_mode");
            this->declare_parameter<float>("startup_delay");

            mDemandRPM = this->create_publisher<std_msgs::msg::String>("demand/rpm", 0);
            mSenseRPM = this->create_publisher<std_msgs::msg::String>("sensor/rpm", 0);
        }

    protected:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mDemandRPM;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mSenseRPM;
    };


    //! Manager for multiple VESC motors

    class MultiVescManager : public rclcpp::Node
    {
    public:
        MultiVescManager()
          : Node("multi_vesc_driver")
        {
            this->declare_parameter<std::vector<std::string> >("motors");

            auto motors = this->get_parameter("motors").as_string_array();

            for (auto motor : motors)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Creating motor %s", motor.c_str());
                mMotors.push_back(std::make_shared<VescMotor>(motor));
            }
        }

    private:
        std::vector<std::shared_ptr<VescMotor> > mMotors;
    };
}



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<multivesc_driver::MultiVescManager>());
    rclcpp::shutdown();
    return 0;
}
