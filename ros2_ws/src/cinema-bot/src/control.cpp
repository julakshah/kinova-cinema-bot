#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Controller : rclcpp::Node {
public:
  Controller() : Node("controller"), count_(0) {
    timer_ =
        this->create_wall_timer(500ms, std::bind(&Controller::run_loop, this));
  }

private:
  void run_loop() {}
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::sping(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}
