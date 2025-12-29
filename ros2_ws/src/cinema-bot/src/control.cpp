#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
using namespace std::chrono_literals;
using namespace std::placeholders; // ADDED

class Controller : public rclcpp::Node {
public:
  Controller() : Node("controller"), count_(0) {
    subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "target_pose", 5,
        std::bind(&Controller::topic_callback, this, _1)); // ADDED ;
    timer_ =
        this->create_wall_timer(500ms, std::bind(&Controller::run_loop, this));
  }

private:
  void run_loop() {}

  void topic_callback(const geometry_msgs::msg::Pose::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "Position: [x: %.2f, y: %.2f, z: %.2f]",
                msg->position.x, msg->position.y,
                msg->position.z); // CHANGED msg. to msg->
  }

  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr
      subscription_; // FIXED :: and ; and capital S
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}
