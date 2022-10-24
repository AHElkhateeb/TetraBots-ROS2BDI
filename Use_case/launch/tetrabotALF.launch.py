import sys
from ament_index_python.packages import get_package_share_directory

ros2_bdi_bringup_dir = get_package_share_directory('ros2_bdi_bringup')
sys.path.append(ros2_bdi_bringup_dir + '/launch/')
from bdi_agent import AgentLaunchDescription
from bdi_agent_skills import AgentAction
from bdi_agent_skills import AgentSensor

def generate_launch_description():
    AGENT_ID = 'tetrabotALF'
    AGENT_GROUP_ID = 'transporting_agents'

    Use_case_share_dir = get_package_share_directory('Use_case')

    # perform moving to
    moveto = AgentAction(
        package='Use_case',
        executable='moveto',
        name='moveto'
    )

    # perform moving to cooperatively
    moveto_cooperatively = AgentAction(
        package='Use_case',
        executable='moveto_cooperatively',
        name='moveto_cooperatively'
    )

    # perform charge
    charge = AgentAction(
        package='Use_case',
        executable='charge',
        name='charge'
    )

    # perform pickup
    pickup = AgentAction(
        package='Use_case',
        executable='pickup',
        name='pickup'
    )

    # perform pickup_cooperatively
    pickup_cooperatively = AgentAction(
        package='Use_case',
        executable='pickup_cooperatively',
        name='pickup_cooperatively'
    )

    # perform drop
    drop = AgentAction(
        package='Use_case',
        executable='drop',
        name='drop'
    )

    # perform drop_cooperatively
    drop_cooperatively = AgentAction(
        package='Use_case',
        executable='drop_cooperatively',
        name='drop_cooperatively'
    )

    # perform change_tool
    change_tool = AgentAction(
        package='Use_case',
        executable='change_tool',
        name='change_tool'
    )

    # sense battery_sensor
    battery_sensor = AgentSensor(
        package='Use_case',
        executable='battery_sensor',
        name='battery_sensor'
    )

    # sense position_sensor
    position_sensor = AgentSensor(
        package='Use_case',
        executable='position_sensor',
        name='position_sensor'
    )

    # sense tool_sensor
    tool_sensor = AgentSensor(
        package='Use_case',
        executable='tool_sensor',
        name='tool_sensor'
    )

    ld = AgentLaunchDescription(
        agent_id=AGENT_ID,
        agent_group=AGENT_GROUP_ID,
        init_params={
            'pddl_file': Use_case_share_dir + '/pddl/transporting-agent.pddl',
            'init_bset': Use_case_share_dir + '/launch/tetrabotALF/init_bset.yaml',
            'init_dset': Use_case_share_dir + '/launch/tetrabotALF/init_dset.yaml',
            'init_reactive_rules_set': Use_case_share_dir + '/launch/tetrabotALF/init_rrules.yaml',
            'debug_log_active': ['belief_manager', 'plan_director', 'event_listener']
        },
        actions=[moveto, moveto_cooperatively, charge, pickup, pickup_cooperatively, drop, drop_cooperatively, change_tool],
        sensors=[battery_sensor, position_sensor, tool_sensor],
        run_only_psys2=False
    ) 

    return ld