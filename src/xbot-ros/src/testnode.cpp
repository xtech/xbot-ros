#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "xbot-service-interface/XbotServiceInterface.hpp"
#include <sensor_msgs/msg/imu.hpp>

#include "ImuServiceInterfaceBase.hpp"


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses a fancy C++11 lambda
* function to shorten the callback syntax, at the expense of making the
* code somewhat more difficult to understand at first glance. */

class MinimalPublisher : public rclcpp::Node, public ImuServiceInterfaceBase
{
private:
    void OnAxesChanged(const double *new_value, uint32_t length) override {
        if (length < 6) {
            return;
        }
        sensor_msgs::msg::Imu message{};
        message.angular_velocity.x = new_value[0];
        message.angular_velocity.y = new_value[1];
        message.angular_velocity.z = new_value[2];
        message.linear_acceleration.x = new_value[3];
        message.linear_acceleration.y = new_value[4];
        message.linear_acceleration.z = new_value[5];
        publisher_->publish(message);
    }

public:
    MinimalPublisher(const xbot::serviceif::Context& ctx)
        : Node("imu"), ImuServiceInterfaceBase(4, ctx) {
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{

    xbot::serviceif::Context ctx = xbot::serviceif::Start(true, "172.16.78.2");

    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalPublisher>(ctx);
    node->Start();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}