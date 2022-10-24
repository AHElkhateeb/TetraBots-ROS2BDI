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
using BDIManaged::ManagedParam;

class BatterySensor : public Sensor
{
    public:
        BatterySensor(const string& sensor_name, const Belief& proto_belief)
        : Sensor(sensor_name, proto_belief, true, false)
        {
            robot_name_ = this->get_parameter("agent_id").as_string();

            TA_bot_status_subscriber_ = this->create_subscription<BotStatus>("/ttb/"+robot_name_+"/statusBot", 
                rclcpp::QoS(10).reliable(),                                                                       //made QoS reliable because not as frequent as position
                std::bind(&BatterySensor::batteryStatusCallback, this, _1));
            
            current_battery_charge_ = Belief();
            current_battery_charge_.name = proto_belief.name;
            current_battery_charge_.pddl_type = proto_belief.pddl_type;
            current_battery_charge_.params = proto_belief.params;
            current_battery_charge_.params[0] = robot_name_;
            current_battery_charge_.value = 0;
        }
        
    private:
        void batteryStatusCallback(const BotStatus::SharedPtr msg)
        {
            bot_status_ = *msg;

            if(bot_status_.bot_akku_status != current_battery_charge_.value)
            {
                sense(current_battery_charge_, DEL);
                current_battery_charge_.value = bot_status_.bot_akku_status;
                sense(current_battery_charge_, ADD);
            }
        }

        string robot_name_;
        Belief current_battery_charge_;
        BotStatus bot_status_;
        rclcpp::Subscription<BotStatus>::SharedPtr TA_bot_status_subscriber_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  Belief b_proto = (ManagedBelief::buildMBFunction("battery_charge", {ManagedParam{"?r","robot"}}, 0)).toBelief();
  auto node = std::make_shared<BatterySensor>("battery_sensor", b_proto);
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}