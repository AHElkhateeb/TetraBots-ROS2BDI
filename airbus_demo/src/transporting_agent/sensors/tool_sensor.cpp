#include <memory>
#include <vector>

#include "ros2_bdi_skills/sensor.hpp"
#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "rclcpp/rclcpp.hpp"

#include "bridge_interfaces/msg/bot_status.hpp"

using std::string;
using std::shared_ptr;
using std::bind;
using std::placeholders::_1;

using ros2_bdi_interfaces::msg::Belief;         
using bridge_interfaces::msg::BotStatus;

using BDIManaged::ManagedBelief;

class ToolSensor : public Sensor
{
    public:
        ToolSensor(const string& sensor_name, const Belief& proto_belief)
        : Sensor(sensor_name, proto_belief, true, false)
        {
            robot_name_ = this->get_parameter("agent_id").as_string();

            TA_bot_status_subscriber_ = this->create_subscription<BotStatus>("/ttb/"+robot_name_+"/statusBot", 
                rclcpp::QoS(10).reliable(),                                                                       //made QoS reliable because not as frequent as position
                std::bind(&ToolSensor::toolStatusCallback, this, _1));
            
            current_tool_ = Belief();
            current_tool_.name = proto_belief.name;
            current_tool_.pddl_type = proto_belief.pddl_type;
            current_tool_.params = proto_belief.params;
            current_tool_.params[0] = robot_name_;
            current_tool_.params[1] = "no_tool";
        }
        
    private:
        void toolStatusCallback(const BotStatus::SharedPtr msg)
        {
            bot_status_ = *msg;

            if(bot_status_.bot_attached_tool_name != current_tool_.params[1])
            {
                sense(current_tool_, DEL);
                current_tool_.params[1] = bot_status_.bot_attached_tool_name;
                sense(current_tool_, ADD);
            }
        }

        string robot_name_;
        Belief current_tool_;
        BotStatus bot_status_;
        rclcpp::Subscription<BotStatus>::SharedPtr TA_bot_status_subscriber_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  Belief b_proto = (ManagedBelief::buildMBPredicate("tool_mounted", {"", ""})).toBelief();
  auto node = std::make_shared<ToolSensor>("tool_sensor", b_proto);
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}