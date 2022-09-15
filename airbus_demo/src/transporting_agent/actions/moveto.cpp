#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "ros2_bdi_skills/bdi_action_executor.hpp"

#include "bridge_interfaces/msg/t265_pos_and_ori.hpp"
#include "bridge_interfaces/msg/team_command.hpp"

#define MEANINGFUL_DIFF 0.001

using bridge_interfaces::msg::TeamCommand;
using bridge_interfaces::msg::T265PosAndOri;
using namespace std;

class MoveTo : public BDIActionExecutor
{
    public:
        MoveTo()
        : BDIActionExecutor("transporting_agent_move", 4)
        {
            robot_name_ = this->get_parameter("agent_id").as_string();
            move_transpoting_agent_cmd_publisher_ = this->create_publisher<TeamCommand>("/ttb/"+robot_name_+"/ext/teamCommand", 
                rclcpp::QoS(1).keep_all());
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
            on_activate(const rclcpp_lifecycle::State & previous_state)
        {
            move_transpoting_agent_cmd_publisher_->on_activate();
            
            transpoting_agent_position_status_subscriber_ = this->create_subscription<T265PosAndOri>("/ttb/"+robot_name_+"/posAndOri", 
                rclcpp::QoS(5).best_effort(),
                std::bind(&MoveTo::positionStatusCallback, this, std::placeholders::_1));

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
            //:parameters (?r - robot ?wp_from ?wp_to - waypoint)
            float step_progress = 0.0f;
            wp_from_ = getArguments()[1];
            wp_to_ = getArguments()[2];
            if (progress_ <= 0.95) 
            {
                auto msg = TeamCommand();
                msg.command_name = "Drive_to";
                msg.command_wait_mode = "";
                msg.bots_affected = robot_name_;
                msg.command_identifier = {robot_name_ + "action_cmd"};
                msg.command_values = get_coordinates(wp_to_);
                msg.command_strings = {"ON", "NO_STOP", "NO_ABORT_STOP"};
                msg.coord_name = "";
                msg.coord_values = {};
                msg.report_goal_reached = false;
                
                move_transpoting_agent_cmd_publisher_->publish(msg);
                step_progress = progress_ - last_step_progress_;            
            }
            else 
            {
                step_progress = progress_ - last_step_progress_;
                if(progress_ == 1.0 || step_progress < MEANINGFUL_DIFF)
                    execSuccess();
            }
            last_step_progress_ = progress_;
            return step_progress;            
        }

    private:

        void positionStatusCallback(const T265PosAndOri::SharedPtr msg)
        {
            float x_current = (*msg).cor_pos_ori[0];
            float y_current = (*msg).cor_pos_ori[1];
            float x_to = get_coordinates(wp_to_)[0];
            float y_to = get_coordinates(wp_to_)[1];
            float x_from = get_coordinates(wp_from_)[0];
            float y_from = get_coordinates(wp_from_)[1];

            progress_ = sqrt(pow((x_current-x_to),2)+pow((y_current-y_to),2))/sqrt(pow((x_from-x_to),2)+pow((y_from-y_to),2));
        }
        
        std::vector<float> get_coordinates(string wp_to)
        {
            if(wp_to == "wp_equip")
                return {1000, 1000, 0.0, 0.0, 0.0, 0.0, 0.25, 0.60, 0.20, 10.0, 1.0, 50.0, 1.0, 45.0};
            else if(wp_to == "wp_pipe")
                return {1000, 4000, 0.0, 0.0, 0.0, 0.0, 0.25, 0.60, 0.20, 10.0, 1.0, 50.0, 1.0, 45.0};
            else if(wp_to == "wp_charge")
                return {6500, 500, 0.0, 0.0, 0.0, 0.0, 0.25, 0.60, 0.20, 10.0, 1.0, 50.0, 1.0, 45.0};
            else if(wp_to == "wp_toolchange")
                return {3500, 500, 0.0, 0.0, 0.0, 0.0, 0.25, 0.60, 0.20, 10.0, 1.0, 50.0, 1.0, 45.0};
            else if(wp_to == "wp_seat")
                return {9000, 1000, 0.0, 0.0, 0.0, 0.0, 0.25, 0.60, 0.20, 10.0, 1.0, 50.0, 1.0, 45.0};
            else if(wp_to == "wp_fuselage")
                return {9000, 4000, 0.0, 0.0, 0.0, 0.0, 0.25, 0.60, 0.20, 10.0, 1.0, 50.0, 1.0, 45.0};
            else
                return {(float)((wp_to[3] - '0')*1000 + 500), (float)((wp_to[4] - '0')*1000 + 500), 0.0, 0.0, 0.0, 0.0, 0.25, 0.60, 0.20, 10.0, 1.0, 50.0, 1.0, 45.0};
        }

        rclcpp_lifecycle::LifecyclePublisher<TeamCommand>::SharedPtr move_transpoting_agent_cmd_publisher_;
        rclcpp::Subscription<T265PosAndOri>::SharedPtr transpoting_agent_position_status_subscriber_;
        float progress_;
        float last_step_progress_;
        std::string robot_name_;
        std::string wp_from_;
        std::string wp_to_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto actionNode = std::make_shared<MoveTo>();
  rclcpp::spin(actionNode->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
