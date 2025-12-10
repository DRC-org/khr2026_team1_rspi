#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class ESP32Test : public rclcpp::Node {
 public:
  ESP32Test() : Node("arduino_test"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("topic", 10);
    auto timer_callback = [this]() -> void {
      auto message = std_msgs::msg::Int32();
      message.data = this->count_++ % 5 + 1;

      RCLCPP_INFO(this->get_logger(), "Publishing: %d", message.data);
      this->publisher_->publish(message);
    };
    timer_ = this->create_wall_timer(1000ms, timer_callback);
  }

 private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ESP32Test>());
  rclcpp::shutdown();
  return 0;
}
