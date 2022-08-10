/*
    implementation for this action



    if progress == 0
        picked_agents_ = select_rnd_agents(num)
        for all picked_agents_:
            accepted, performed = sendBeliefUpd tool_required
            accepted, performed = sendBeliefUpd payload_in (i.e. where it is now)
            
            if not accepted or not performed:
                FAILURE
            else:
                accepted, performed = sendDesireReq with value: 
                    [payload_in (where it should go), ***something to say that you need to wait and do the job with others***]
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

#include "ros2_bdi_skills/bdi_action_executor.hpp"
#include "rclcpp/rclcpp.hpp"

class AskForTransportation : public BDIActionExecutor
{
    public:
        AskForTransportation()
        : BDIActionExecutor("ask_for_transportation", 4)
        {}

        float advanceWork()
        {
            //should implement here the step logic for your action, returning a value in [0-1] which specify the step progress
            return 0.0084f;            
        }

    private:
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto actionNode = std::make_shared<AskForTransportation>();
  rclcpp::spin(actionNode->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}