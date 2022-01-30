/**
 *  @file   main.cpp
 *  @date   2021-01-30
 *  @author Yadunund Vijay
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>

#include <std_msgs/msg/string.hpp>

#include <thread>

//##############################################################################
class Listener : public rclcpp::Node
{
public:
  using String = std_msgs::msg::String;

  Listener(std::size_t id)
  : Node("Listener_" + std::to_string(id)),
    _id(id)
  {
    // Callback group that the multi-threaded executor looks for
    _callback_group_subscriber = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    // Each of these callback groups is basically a thread
    // Everything assigned to one of them gets bundled into the same thread
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = _callback_group_subscriber;

    _sub = this->create_subscription<String>(
      "chatter",
      1,
      [this](const String::SharedPtr msg)
      {

        const auto hashed = std::hash<std::thread::id>()(
          std::this_thread::get_id());
        const std::string thread_name = std::to_string(hashed);
        RCLCPP_INFO(
          this->get_logger(),
          "Listener[%ld] on thread[%s]: Received %s",
          _id, thread_name.c_str(), msg->data.c_str());
      },
      sub_opt
    );

    RCLCPP_INFO(
      this->get_logger(),
      "Initialized Listener[%ld]",
      _id);
  }

private:
  std::size_t _id;
  rclcpp::CallbackGroup::SharedPtr _callback_group_subscriber;
  rclcpp::Subscription<String>::SharedPtr _sub;
};

//##############################################################################
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  if (argc < 2)
  {
    throw std::runtime_error("Usage: ros2 run test_multithreaded_executor "
    "test_executor NUM_THREADS");
  }

  const std::size_t num_subs = std::atoi(argv[1]);

  rclcpp::executors::MultiThreadedExecutor executor{
    rclcpp::ExecutorOptions(), num_subs};

  std::vector<rclcpp::Node::SharedPtr> nodes;

  for (std::size_t i = 0; i < num_subs; ++i)
    nodes.push_back(std::make_shared<Listener>(i));

  for (const auto& node : nodes)
    executor.add_node(node);

  executor.spin();
  rclcpp::shutdown();
}