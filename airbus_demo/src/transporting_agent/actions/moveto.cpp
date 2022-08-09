#include "cmath"

#include "rclcpp/rclcpp.hpp"
#include "ros2_bdi_skills/bdi_action_executor.hpp"

#include "webots_ros2_simulations_interfaces/msg/move_status.hpp"
#include "bridge_interfaces/msg/team_command.hpp"

#define MEANINGFUL_DIFF 0.001

using bridge_interfaces::msg::TeamCommand;
using webots_ros2_simulations_interfaces::msg::MoveStatus;

class MoveTo : public BDIActionExecutor
{
    public:
        MoveTo()
        : BDIActionExecutor("transpoting_agent_move", 4)
        {
            robot_name_ = this->get_parameter("agent_id").as_string();
            move_transpoting_agent_cmd_publisher_ = this->create_publisher<TeamCommand>("/ttb/"+robot_name_+"/ext/teamCommand", 
                rclcpp::QoS(1).keep_all());
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
            on_activate(const rclcpp_lifecycle::State & previous_state)
        {
            move_transpoting_agent_cmd_publisher_->on_activate();
            
            carrier_move_status_subscriber_ = this->create_subscription<MoveStatus>("/"+robot_name_+"/move_status", 
                rclcpp::QoS(5).best_effort(),
                std::bind(&MoveTo::carrierMoveStatusCallback, this, std::placeholders::_1));

            last_step_progress_info_ = 0.0f;

            return BDIActionExecutor::on_activate(previous_state);
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
            on_deactivate(const rclcpp_lifecycle::State & previous_state)
        {
            move_transpoting_agent_cmd_publisher_->on_deactivate();

            return BDIActionExecutor::on_deactivate(previous_state);
        }

        float advanceWork()
        {
            float step_progress = 0.0f;
            std::string commandName = getArguments()[0];
            std::string destination = getArguments()[4];
            if(last_step_progress_info_ > 0.0 && destination != move_status_.target_name)//action execution had started, but it somehow was interrupted by some other controllers
                execFailed("Target position for the action was '" + destination + "' and now has been changed to '" + move_status_.target_name + "'!");
            
            else if (last_step_progress_info_ == 0.0 && destination != move_status_.target_name)//move cmd to trigger action execution hasn't been given yet 
            {
                auto msg = TeamCommand();
                msg.command_name = commandName;
                msg.bots_affected = robot_name_;
                msg.command_values = {std::stof(destination),0,0};
                move_transpoting_agent_cmd_publisher_->publish(msg);
            }
            else if (destination == move_status_.target_name) 
            {
                step_progress = move_status_.progress - last_step_progress_info_;
                last_step_progress_info_ = move_status_.progress;

                if(move_status_.progress == 1.0 || move_status_.progress > 0.95 && step_progress < MEANINGFUL_DIFF)
                    execSuccess();
            }
            
            return step_progress;            
        }

    private:

        void carrierMoveStatusCallback(const MoveStatus::SharedPtr msg)
        {
            move_status_ = *msg;
        }

        rclcpp_lifecycle::LifecyclePublisher<TeamCommand>::SharedPtr move_transpoting_agent_cmd_publisher_;
        rclcpp::Subscription<MoveStatus>::SharedPtr carrier_move_status_subscriber_;
        float last_step_progress_info_;
        MoveStatus move_status_;
        std::string robot_name_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto actionNode = std::make_shared<MoveTo>();
  rclcpp::spin(actionNode->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
