#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <ctime>
#include <sstream>
#include <iomanip>

class rtc_keeper : public rclcpp::Node
{
public:
    rtc_keeper()
        : Node("rtc_keeper")  // Node name
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("rtc_time", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&rtc_keeper::publish_rtc_time, this));
    }

private:
    void publish_rtc_time()
    {
        std::time_t now = std::time(nullptr);
        std::tm *rtc_time = std::localtime(&now);

        std::ostringstream time_stream;
        time_stream << std::put_time(rtc_time, "%Y-%m-%d %H:%M:%S");

        auto message = std_msgs::msg::String();
        message.data = time_stream.str();
        RCLCPP_INFO(this->get_logger(), "Publishing RTC time: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rtc_keeper>());
    rclcpp::shutdown();
    return 0;
}
