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

class StatusSensor : public Sensor
{
    public:
        StatusSensor(const string& sensor_name, const Belief& proto_belief)
        : Sensor(sensor_name, proto_belief, true, false)
        {
            robot_name_ = this->get_parameter("agent_id").as_string();

            TA_bot_status_subscriber_ = this->create_subscription<BotStatus>("/ttb/"+robot_name_+"/statusBot", 
                rclcpp::QoS(10).reliable(),                                                                       //made QoS reliable because not as frequent as position
                std::bind(&StatusSensor::StatusCallback, this, _1));
            
            current_status_ = Belief();
            current_status_.name = proto_belief.name;
            current_status_.pddl_type = proto_belief.pddl_type;
            current_status_.params = proto_belief.params;
            current_status_.params[0] = robot_name_;
        }
        
    private:
        void StatusCallback(const BotStatus::SharedPtr msg)
        {
            bot_status_ = *msg;

            if(bot_status_.bot_status == "transport" || bot_status_.bot_status == "load" || bot_status_.bot_status == "defect")
            {
                sense(current_status_, DEL);
            }
            else if(bot_status_.bot_status == "charge" || bot_status_.bot_status == "move")
            {
                sense(current_status_, ADD);
            }
        }

        string robot_name_;
        Belief current_status_;
        BotStatus bot_status_;
        rclcpp::Subscription<BotStatus>::SharedPtr TA_bot_status_subscriber_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  Belief b_proto = (ManagedBelief::buildMBPredicate("free", {""})).toBelief();
  auto node = std::make_shared<StatusSensor>("status_sensor", b_proto);
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}