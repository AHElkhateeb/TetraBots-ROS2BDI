#include <memory>
#include <vector>

#include "ros2_bdi_skills/sensor.hpp"
#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "rclcpp/rclcpp.hpp"

#include "bridge_interfaces/msg/job_definition.hpp"

using std::string;
using std::to_string;
using std::shared_ptr;
using std::bind;
using std::placeholders::_1;

using ros2_bdi_interfaces::msg::Belief;         
using bridge_interfaces::msg::JobDefinition;

using BDIManaged::ManagedBelief;

class PayloadShouldBeInSensor : public Sensor
{
    public:
        PayloadShouldBeInSensor(const string& sensor_name, const Belief& proto_belief)
        : Sensor(sensor_name, proto_belief, true, false)
        {
            robot_name_ = this->get_parameter("agent_id").as_string();

            OA_payload_subscriber_ = this->create_subscription<JobDefinition>("/ttb/central/transportJob", 
                rclcpp::QoS(5).reliable(),
                std::bind(&PayloadShouldBeInSensor::jobReceivedCallback, this, _1));
            
            payload_belief = Belief();
            payload_belief.name = proto_belief.name;
            payload_belief.pddl_type = proto_belief.pddl_type;
            payload_belief.params = proto_belief.params;
            payload_belief.params[0] = "";
            payload_belief.params[1] = "";
            payload_belief.params[2] = "";
            payload_belief.params[3] = "";
        }
        
    private:
        void jobReceivedCallback(const JobDefinition::SharedPtr msg)
        {
            new_payload_ = *msg;

            /*
            float payload_x = new_payload_.start_location[0];
            float payload_y = new_payload_.start_location[1];
            string raw_wp = "wp_" + to_string((int)payload_x) + to_string((int)payload_y) ;
            if(raw_wp == "wp_00")
            	raw_wp = "wp_equip";
            else if(raw_wp == "wp_06")
            	raw_wp = "wp_pipe";
            else if(raw_wp == "wp_20")
            	raw_wp = "wp_charge";
            else if(raw_wp == "wp_03")
            	raw_wp = "wp_toolchange";
            else if(raw_wp == "wp_60")
            	raw_wp = "wp_seat";
            else if(raw_wp == "wp_66")
            	raw_wp = "wp_fuselage";
            */

            payload_belief.params[0] = new_payload_.payload;
            payload_belief.params[1] = new_payload_.destination_location_name;
            payload_belief.params[2] = new_payload_.team.required_tool[0];
            payload_belief.params[3] = new_payload_.team.number_of_bots;
            sense(payload_belief, ADD);      
        }

        string robot_name_;
        Belief payload_belief;
        JobDefinition new_payload_;
        rclcpp::Subscription<JobDefinition>::SharedPtr OA_payload_subscriber_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  Belief b_proto = (ManagedBelief::buildMBPredicate("payload_should_be_in", {"", "", "", ""})).toBelief();
  auto node = std::make_shared<PayloadShouldBeInSensor>("payload_should_be_in_sensor", b_proto);
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
