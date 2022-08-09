#include <memory>
#include <vector>

#include "ros2_bdi_skills/sensor.hpp"
#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "rclcpp/rclcpp.hpp"

#include "bridge_interfaces/msg/t265_pos_and_ori.hpp"

using std::string;
using std::to_string;
using std::shared_ptr;
using std::bind;
using std::placeholders::_1;

using ros2_bdi_interfaces::msg::Belief;         
using bridge_interfaces::msg::T265PosAndOri;

using BDIManaged::ManagedBelief;

class TransportingAgentWPSensor : public Sensor
{
    public:
        TransportingAgentWPSensor(const string& sensor_name, const Belief& proto_belief)
        : Sensor(sensor_name, proto_belief, true, false)
        {
            robot_name_ = this->get_parameter("agent_id").as_string();

            TA_position_status_subscriber_ = this->create_subscription<T265PosAndOri>("/ttb/"+robot_name_+"/posAndOri", 
                rclcpp::QoS(5).best_effort(),
                std::bind(&TransportingAgentWPSensor::positionStatusCallback, this, _1));
            
            current_wp_ = Belief();
            current_wp_.name = proto_belief.name;
            current_wp_.pddl_type = proto_belief.pddl_type;
            current_wp_.params = proto_belief.params;
            current_wp_.params[0] = robot_name_;
            current_wp_.params[1] = "charge_wp";
        }
        
    private:
        void positionStatusCallback(const T265PosAndOri::SharedPtr msg)
        {
            position_status_ = *msg;

            float x = position_status_.cor_pos_ori[0];
            float y = position_status_.cor_pos_ori[1];

            string raw_wp = "wp_" + to_string((int)x) + to_string((int)y) ;

            if(raw_wp == "00_wp")
            	raw_wp = "equip_wp";
            else if(raw_wp == "06_wp")
            	raw_wp = "pipe_wp";
            else if(raw_wp == "20_wp")
            	raw_wp = "charge_wp";
            else if(raw_wp == "36_wp")
            	raw_wp = "toolchange_wp";
            else if(raw_wp == "60_wp")
            	raw_wp = "seat_wp";
            else if(raw_wp == "66_wp")
            	raw_wp = "fuselage_wp";

            sense(current_wp_, DEL);
            current_wp_.params[1] = raw_wp;
            sense(current_wp_, ADD);      
        }

        string robot_name_;
        Belief current_wp_;
        T265PosAndOri position_status_;
        rclcpp::Subscription<T265PosAndOri>::SharedPtr TA_position_status_subscriber_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  Belief b_proto = (ManagedBelief::buildMBPredicate("in", {"", ""})).toBelief();
  auto node = std::make_shared<TransportingAgentWPSensor>("wp_sensor", b_proto);
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
