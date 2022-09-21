/*
    implementation for this action



    if progress == 0
        picked_agents_ = select_rnd_agents(num)
        for all picked_agents_:
            accepted, performed = sendBeliefUpd (tool_required payload_x tool_x)
            accepted, performed = sendBeliefUpd (payload_in wp_from)
            accepted, performed = sendBeliefUpd (transport_cooperatively r r1 r2)

            if not accepted or not performed:
                FAILURE
            else:
                accepted, performed = sendDesireReq with value: (payload_in wp_to)
                if not accepted or not performed:
                    FAILURE
                    
    else:
        target_reached = .... #isMonitoredDesireFulfilled true for all the monitored desires

        if(target_reached)
            SUCCESS
        elif(progress == 99 and not target_reached):
            FAILURE

    
    return tiny_increment 

*/

#include "rclcpp/rclcpp.hpp"
#include "ros2_bdi_skills/bdi_action_executor.hpp"
#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/desire.hpp"

using ros2_bdi_interfaces::msg::Belief;
using ros2_bdi_interfaces::msg::Desire;
using BDICommunications::UpdDesireResult;

class AskForTransportation : public BDIActionExecutor
{
    public:
        AskForTransportation()
        : BDIActionExecutor("ask_for_transportation", 1) , robots_({"ta1", "ta2", "ta3", "ta4"})
        {//:parameters (?p - payload ?wp_from ?wp_to - waypoint ?t - tool ?num - number)
            payload_ = getArguments()[0];
            wp_from_ = getArguments()[1];
            wp_to_ = getArguments()[2];
            tool_required_ = getArguments()[3];

            if (getArguments()[4] == "one") robots_num_ = 1;
            else if (getArguments()[4] == "two") robots_num_ = 2;
            else if (getArguments()[4] == "three") robots_num_ = 3;
            else execFailed("Wrong number of robots is requested");
        }

        float advanceWork()
        {
            static int desires_sent = 0;
            float step_progress = 0.0;
            float action_progress = getProgress();

            if(action_progress < 0.1)
            {
                BDICommunications::UpdBeliefResult result0[robots_num_];
                BDICommunications::UpdBeliefResult result1[robots_num_];
                BDICommunications::UpdBeliefResult result2[robots_num_];
                BDICommunications::UpdDesireResult result[robots_num_];
                for(int i = 0; i < robots_num_; i++)
                {
                    result0[i] = sendUpdBeliefRequest(robots_[i],buildPayloadInBelief(payload_,wp_from_),BDICommunications::ADD);
                    result1[i] = sendUpdBeliefRequest(robots_[i],buildToolRequiredBelief(payload_,tool_required_),BDICommunications::ADD);
                    switch (robots_num_)
                    {
                        case 1:
                            result2[i] = sendUpdBeliefRequest(robots_[i],buildTransportCooperativelyBelief(robots_[i],"",""),BDICommunications::ADD);
                            break;
                        case 2:
                            result2[i] = sendUpdBeliefRequest(robots_[i],buildTransportCooperativelyBelief(robots_[i],robots_[(i+1)%2],""),BDICommunications::ADD);
                            break;
                        case 3:
                            result2[i] = sendUpdBeliefRequest(robots_[i],buildTransportCooperativelyBelief(robots_[i],robots_[(i+1)%3],robots_[(i+2)%3]),BDICommunications::ADD);
                    }
                    if(result0[i].performed && result1[i].performed && result2[i].performed && result0[i].accepted && result1[i].accepted && result2[i].accepted)
                    {
                        requested_desire_ = buildDesire(payload_,wp_to_);
                        result[i] = sendUpdDesireRequest(robots_[i], requested_desire_, BDICommunications::ADD, true);
                        if (result[i].desire.name == requested_desire_.name)
                            desires_sent += (result[i].accepted && result[i].performed)? 1 : 0;
                        else
                            execFailed("Couldn't add desire to transporting agent no. " + (i+1));
                    }
                    else
                        execFailed("Couldn't add beliefs to transporting agent no. " + (i+1));
                }

                if(desires_sent == robots_num_)
                    step_progress += 0.1;
                else
                    execFailed("Couldn't add desire to enough transporting agents");
            }
            else if (action_progress >= 0.98)
                execFailed("Too much time waited for the transporting agents to fulfill the desire");
            else
            {
                int desires_fulfilled_= 0;
                for(int i = 0; i < desires_sent; i++)
                {
                    if (isMonitoredDesireFulfilled(robots_[i], requested_desire_))
                    desires_fulfilled_++;
                }
                
                if(desires_fulfilled_==desires_sent)
                    execSuccess();
                else
                    step_progress += 0.01; //desire request already made and should be finished within 88 seconds
            }

            return step_progress;            
        }

    private:

        Belief buildPayloadInBelief(const std::string& payload, const std::string& waypoint)
        {
            auto belief = Belief();
            belief.name = "payload_in";
            belief.pddl_type = belief.PREDICATE_TYPE;
            belief.params = {payload, waypoint};
            return belief;
        }

        Belief buildToolRequiredBelief(const std::string& payload, const std::string& tool)
        {
            auto belief = Belief();
            belief.name = "tool_required";
            belief.pddl_type = belief.PREDICATE_TYPE;
            belief.params = {payload, tool};
            return belief;
        }

        Belief buildTransportCooperativelyBelief(const std::string& r, const std::string& r1, const std::string& r2)
        {
            auto belief = Belief();
            belief.name = "transport_cooperatively";
            belief.pddl_type = belief.PREDICATE_TYPE;
            belief.params = {r, r1, r2};
            return belief;
        }

        Desire buildDesire(const std::string& payload, const std::string& waypoint)
        {
            auto desire = Desire();
            desire.name = "Transport_" + payload;
            desire.deadline = 6.0;
            desire.priority = 0.6;
            auto value = Belief();
            {
                value.name = "payload_in";
                value.pddl_type = value.PREDICATE_TYPE;
                value.params = {payload, waypoint};
            }
            desire.value = {value};
            return desire;
        }

        float progress_;
        float last_step_progress_;
        
        Desire requested_desire_;
        std::string robots_[4];
        std::string payload_;
        std::string wp_from_;
        std::string wp_to_;
        std::string tool_required_;
        int robots_num_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto actionNode = std::make_shared<AskForTransportation>();
  rclcpp::spin(actionNode->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}