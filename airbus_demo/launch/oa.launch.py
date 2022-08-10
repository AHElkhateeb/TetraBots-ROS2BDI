import sys
from ament_index_python.packages import get_package_share_directory

ros2_bdi_bringup_dir = get_package_share_directory('ros2_bdi_bringup')
sys.path.append(ros2_bdi_bringup_dir + '/launch/')
from bdi_agent import AgentLaunchDescription
from bdi_agent_skills import AgentAction
from bdi_agent_skills import AgentSensor

def generate_launch_description():
    AGENT_ID = 'oa'
    AGENT_GROUP_ID = 'organizing_agents'

    airbus_demo_share_dir = get_package_share_directory('airbus_demo')

    # perform ask_for_transportation
    ask_for_transportation = AgentAction(
        package='airbus_demo',
        executable='ask_for_transportation',
        name='ask_for_transportation'
    )

    # sense payload_in_sensor
    payload_in_sensor = AgentSensor(
        package='airbus_demo',
        executable='payload_in_sensor',
        name='payload_in_sensor'
    )
    
    # sense payload_should_be_in_sensor
    payload_should_be_in_sensor = AgentSensor(
        package='airbus_demo',
        executable='payload_should_be_in_sensor',
        name='payload_should_be_in_sensor'
    )

    ld = AgentLaunchDescription(
        agent_id=AGENT_ID,
        agent_group=AGENT_GROUP_ID,
        init_params={
            'pddl_file': airbus_demo_share_dir + '/pddl/organizing-agent.pddl',
            'init_bset': airbus_demo_share_dir + '/launch/oa/init_bset.yaml',
            'init_dset': airbus_demo_share_dir + '/launch/oa/init_dset.yaml',
            'init_reactive_rules_set': airbus_demo_share_dir + '/launch/oa/init_rrules.yaml',
            'debug_log_active': ['belief_manager', 'scheduler', 'plan_director']
        },
        actions=[ask_for_transportation],
        sensors=[payload_in_sensor, payload_should_be_in_sensor],
        run_only_psys2=False
    ) 

    return ld