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
using BDIManaged::ManagedParam;

class PayloadInSensor : public Sensor
{
    public:
        PayloadInSensor(const string& sensor_name, const Belief& proto_belief)
        : Sensor(sensor_name, proto_belief, true, false)
        {
            robot_name_ = this->get_parameter("agent_id").as_string();

            OA_payload_subscriber_ = this->create_subscription<JobDefinition>("/ttb/central/transportJob", 
                rclcpp::QoS(5).reliable(),
                std::bind(&PayloadInSensor::jobReceivedCallback, this, _1));
            
            payload_belief = Belief();
            payload_belief.name = proto_belief.name;
            payload_belief.pddl_type = proto_belief.pddl_type;
            payload_belief.params = proto_belief.params;
            payload_belief.params[0] = "";
            payload_belief.params[1] = "";
        }
        
    private:
        void jobReceivedCallback(const JobDefinition::SharedPtr msg)
        {
            new_payload_ = *msg;

            payload_belief.params[0] = new_payload_.payload;
            payload_belief.params[1] = new_payload_.start_location_name;
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

  Belief b_proto = (ManagedBelief::buildMBPredicate("payload_in", {ManagedParam{"?p","payload"}, ManagedParam{"?wp","waypoint"}})).toBelief();
  auto node = std::make_shared<PayloadInSensor>("payload_in_sensor", b_proto);
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}