#include "ros2_bdi_skills/bdi_action_executor.hpp"
#include "rclcpp/rclcpp.hpp"

class Charge : public BDIActionExecutor
{
    public:
        Charge()
        : BDIActionExecutor("charge", 4)
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
  
  auto actionNode = std::make_shared<Charge>();
  rclcpp::spin(actionNode->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}