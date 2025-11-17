#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>

class SerialTestNode : public rclcpp::Node
{
public:
    SerialTestNode() : Node("serial_test_node")
    {
        declare_parameter<std::string>("port", "/dev/ttyUSB0");
        declare_parameter<int>("baud", 115200);

        port_ = get_parameter("port").as_string();
        baud_ = get_parameter("baud").as_int();

        try {
            serial_.setPort(port_);
            serial_.setBaudrate(baud_);
            serial::Timeout to = serial::Timeout::simpleTimeout(20);
            serial_.setTimeout(to);
            serial_.open();
        }
        catch (serial::IOException& e) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open port: %s", port_.c_str());
        }

        if (serial_.isOpen()) {
            RCLCPP_INFO(this->get_logger(),
                "Serial port opened successfully: %s @ %d baud", port_.c_str(), baud_);
        } else {
            RCLCPP_ERROR(this->get_logger(),
                "Failed to open serial port: %s", port_.c_str());
        }

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&SerialTestNode::sendTestString, this)
        );
    }

private:
    void sendTestString()
    {
        if (!serial_.isOpen()) {
            RCLCPP_ERROR(this->get_logger(), "Serial port not open!");
            return;
        }

        std::string test_cmd = "TEST 123\n";

        size_t bytes = serial_.write(test_cmd);

        RCLCPP_INFO(this->get_logger(),
                    "Sent: %s   (%ld bytes)", test_cmd.c_str(), bytes);
    }

    serial::Serial serial_;
    std::string port_;
    int baud_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialTestNode>());
    rclcpp::shutdown();
    return 0;
}
