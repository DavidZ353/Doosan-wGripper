#include <chrono>
#include <memory>
#include <thread>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <schunk_gripper_interfaces/srv/move_to_absolute_position.hpp>
#include <schunk_gripper_interfaces/srv/grip.hpp>
#include <schunk_gripper_interfaces/srv/release.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;

class GripperController : public rclcpp::Node {
public:
  GripperController() : Node("gripper_controller"), current_position_(std::nullopt) {
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/schunk/driver/EGU_60_M_B_1/joint_states", 10,
      std::bind(&GripperController::joint_state_callback, this, std::placeholders::_1));

    move_client_ = this->create_client<schunk_gripper_interfaces::srv::MoveToAbsolutePosition>(
      "/schunk/driver/EGU_60_M_B_1/move_to_absolute_position");
    grip_client_ = this->create_client<schunk_gripper_interfaces::srv::Grip>(
      "/schunk/driver/EGU_60_M_B_1/grip");
    release_client_ = this->create_client<schunk_gripper_interfaces::srv::Release>(
      "/schunk/driver/EGU_60_M_B_1/release");
    ack_client_ = this->create_client<std_srvs::srv::Trigger>(
      "/schunk/driver/EGU_60_M_B_1/acknowledge");

    wait_for_services();
  }

  void execute() {
    while (!current_position_.has_value()) {
      RCLCPP_INFO(this->get_logger(), "Waiting for position data...");
      rclcpp::spin_some(this->get_node_base_interface());
      std::this_thread::sleep_for(500ms);
    }

    double start = current_position_.value();
    double close = std::min(0.08, start + 0.03);
    double open = std::max(0.0, start);

    acknowledge_gripper();
    move_gripper(close);
    std::this_thread::sleep_for(500ms);

    acknowledge_gripper();
    move_gripper(open);
    std::this_thread::sleep_for(500ms);

    acknowledge_gripper();
    move_gripper(0.02);
    acknowledge_gripper();
    grip_gripper(50);
    std::this_thread::sleep_for(1s);

    acknowledge_gripper();
    release_gripper();
  }

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (!msg->position.empty()) {
      current_position_ = msg->position[0];
    }
  }

  void wait_for_services() {
    while (!move_client_->wait_for_service(2s)) {
      RCLCPP_INFO(this->get_logger(), "Waiting for move service...");
    }
    while (!grip_client_->wait_for_service(2s)) {
      RCLCPP_INFO(this->get_logger(), "Waiting for grip service...");
    }
    while (!release_client_->wait_for_service(2s)) {
      RCLCPP_INFO(this->get_logger(), "Waiting for release service...");
    }
    while (!ack_client_->wait_for_service(2s)) {
      RCLCPP_INFO(this->get_logger(), "Waiting for acknowledge service...");
    }
  }

  template<typename ServiceT, typename RequestT>
  typename rclcpp::Client<ServiceT>::SharedFuture call_service_with_retry(
    typename rclcpp::Client<ServiceT>::SharedPtr client, RequestT request, const std::string &label, int retries = 3)
  {
    for (int i = 0; i < retries; ++i) {
      auto future = client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 5s) ==
          rclcpp::FutureReturnCode::SUCCESS)
        return future;
      RCLCPP_WARN(this->get_logger(), "%s service call failed. Retrying...", label.c_str());
    }
    RCLCPP_ERROR(this->get_logger(), "%s service call ultimately failed after %d retries.", label.c_str(), retries);
    return {};  // Return invalid future
  }

  void acknowledge_gripper() {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = call_service_with_retry(ack_client_, request, "Acknowledge");
    if (future.valid()) {
      RCLCPP_INFO(this->get_logger(), "Acknowledge: %s", future.get()->message.c_str());
    }
  }

  void move_gripper(double position) {
    auto request = std::make_shared<schunk_gripper_interfaces::srv::MoveToAbsolutePosition::Request>();
    request->position = position;
    request->velocity = 0.02;
    request->use_gpe = false;
    auto future = call_service_with_retry(move_client_, request, "Move");
    if (future.valid()) {
      RCLCPP_INFO(this->get_logger(), "Moved to %.3f: success=%s, msg=%s", position,
                  future.get()->success ? "true" : "false",
                  future.get()->message.c_str());
    }
  }

  void grip_gripper(double force) {
    auto request = std::make_shared<schunk_gripper_interfaces::srv::Grip::Request>();
    request->force = force;
    request->outward = false;
    request->use_gpe = false;
    auto future = call_service_with_retry(grip_client_, request, "Grip");
    if (future.valid()) {
      RCLCPP_INFO(this->get_logger(), "Gripped: success=%s, msg=%s",
                  future.get()->success ? "true" : "false",
                  future.get()->message.c_str());
    }
  }

  void release_gripper() {
    auto request = std::make_shared<schunk_gripper_interfaces::srv::Release::Request>();
    request->use_gpe = false;
    auto future = call_service_with_retry(release_client_, request, "Release");
    if (future.valid()) {
      RCLCPP_INFO(this->get_logger(), "Released: success=%s, msg=%s",
                  future.get()->success ? "true" : "false",
                  future.get()->message.c_str());
    }
  }

  std::optional<double> current_position_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  rclcpp::Client<schunk_gripper_interfaces::srv::MoveToAbsolutePosition>::SharedPtr move_client_;
  rclcpp::Client<schunk_gripper_interfaces::srv::Grip>::SharedPtr grip_client_;
  rclcpp::Client<schunk_gripper_interfaces::srv::Release>::SharedPtr release_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr ack_client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperController>();
  node->execute();
  rclcpp::shutdown();
  return 0;
}
