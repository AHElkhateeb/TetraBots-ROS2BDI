#include "ros2_bdi_skills/bdi_action_executor.hpp"
#include "rclcpp/rclcpp.hpp"

class MoveTo : public BDIActionExecutor
{
    public:
        MoveTo()
        : BDIActionExecutor("moveto", 4)
        {}

        float advanceWork()
        {
            //should implement here the step logic for your action, returning a value in [0-1] which specify the step progress
            return 0.0167f;            
        }

    private:
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto actionNode = std::make_shared<MoveTo>();
  rclcpp::spin(actionNode->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
