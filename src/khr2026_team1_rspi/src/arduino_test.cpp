#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class ArduinoTest : public rclcpp::Node {
 public:
  ArduinoTest() : Node("arduino_test"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("topic", 10);
    auto timer_callback = [this]() -> void {
      auto message = std_msgs::msg::Int32();
      message.data = this->count_++;

      RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.data);
      this->publisher_->publish(message);
    };
    timer_ = this->create_wall_timer(500ms, timer_callback);
  }

 private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArduinoTest>());
  rclcpp::shutdown();
  return 0;
}
