#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "bridge_interfaces/msg/still_alive.hpp"
#include <string>

using std::placeholders::_1;
using bridge_interfaces::msg::StillAlive;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<StillAlive>(
      "/ttb/central/stillAlive", rclcpp::QoS(10).reliable(), std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const StillAlive::SharedPtr msg) const
    {
      RCLCPP_INFO( this->get_logger(), "/ttb/central/stillAlive topic received a message from Bot ID: " + msg->bot_id );
    }
    rclcpp::Subscription<StillAlive>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
