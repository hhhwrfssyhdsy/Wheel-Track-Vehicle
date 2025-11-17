#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <serial/serial.h>

class SerialMotorNode : public rclcpp::Node
{
public:
    SerialMotorNode() : Node("serial_motor_node")
    {
        declare_parameter<std::string>("port", "/dev/ttyUSB0");
        declare_parameter<int>("baud", 115200);
        
        port_ = get_parameter("port").as_string();
        baud_ = get_parameter("baud").as_int();

        serial_.setPort(port_);
        serial_.setBaudrate(baud_);
        serial::Timeout to = serial::Timeout::simpleTimeout(20);
        serial_.setTimeout(to);
        serial_.open();

        if (!serial_.isOpen()) {
            RCLCPP_ERROR(get_logger(), "Failed to open serial port!");
        }

        sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&SerialMotorNode::cmdVelCallback, this, std::placeholders::_1));
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        float linear = msg->linear.x;   // m/s
        float angular = msg->angular.z; // rad/s

        // 转换成左右轮速度（mm/s），假设车宽 300mm
        float wheel_base = 300.0f;
        float v = linear * 1000.0f; // m/s -> mm/s
        float w = angular * 1000.0f;

        float v_left  = v - w * wheel_base / 2;
        //float v_right = v + w * wheel_base / 2;

        // 构造命令
        char cmd_l[64];// cmd_r[64];

        sprintf(cmd_l,  "SPEED 7 %.1f\n", v_left);
        //sprintf(cmd_r,  "SPEED 2 %.1f\n", v_right);

        serial_.write(cmd_l);
        //serial_.write(cmd_r);

        //RCLCPP_INFO(this->get_logger(), "L=%.1f mm/s  R=%.1f mm/s", v_left, v_right);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;

    serial::Serial serial_;
    std::string port_;
    int baud_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialMotorNode>());
    rclcpp::shutdown();
    return 0;
}
