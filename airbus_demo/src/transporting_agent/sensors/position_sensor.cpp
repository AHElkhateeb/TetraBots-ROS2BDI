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
            current_wp_.params[1] = "wp_charge";
        }
        
    private:
        void positionStatusCallback(const T265PosAndOri::SharedPtr msg)
        {
            position_status_ = *msg;

            float x = position_status_.cor_pos_ori[0]/1000;
            float y = position_status_.cor_pos_ori[1]/1000;

            string raw_wp = "wp_" + to_string((int)x) + to_string((int)y) ;

            if(raw_wp == "wp_00" || raw_wp == "wp_10" || raw_wp == "wp_01" || raw_wp == "wp_11")
            	raw_wp = "wp_equip";
            else if(raw_wp == "wp_03" || raw_wp == "wp_04" || raw_wp == "wp_13" || raw_wp == "wp_14")
            	raw_wp = "wp_pipe";
            else if(raw_wp == "wp_60")
            	raw_wp = "wp_charge";
            else if(raw_wp == "wp_30")
            	raw_wp = "wp_toolchange";
            else if(raw_wp == "wp_80" || raw_wp == "wp_90" || raw_wp == "wp_81" || raw_wp == "wp_91")
            	raw_wp = "wp_seat";
            else if(raw_wp == "wp_83" || raw_wp == "wp_93" || raw_wp == "wp_84" || raw_wp == "wp_94")
            	raw_wp = "wp_fuselage";

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
