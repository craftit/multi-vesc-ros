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

    class MultiVescDriver : public rclcpp::Node
    {
    public:
        MultiVescDriver()
                : Node("multi_vesc_driver")
        {
            publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
            timer_ = this->create_wall_timer(
                    500ms, std::bind(&MultiVescDriver::timer_callback, this));
        }

    private:
        void timer_callback()
        {
            auto message = std_msgs::msg::String();
            message.data = "Hello, world!";
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            publisher_->publish(message);
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    };
}



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<multivesc_driver::MultiVescDriver>());
    rclcpp::shutdown();
    return 0;
}
