import sys
from ament_index_python.packages import get_package_share_directory

ros2_bdi_bringup_dir = get_package_share_directory('ros2_bdi_bringup')
sys.path.append(ros2_bdi_bringup_dir + '/launch/')
from bdi_agent import AgentLaunchDescription
from bdi_agent_skills import AgentAction
from bdi_agent_skills import AgentSensor

def generate_launch_description():
    AGENT_ID = 'TA1'
    AGENT_GROUP_ID = 'transporting_agents'

    airbus_demo_share_dir = get_package_share_directory('airbus_demo')

    # perform moving to
    moveto = AgentAction(
        package='airbus_demo',
        executable='moveto',
        name='moveto'
    )

    # perform charge
    charge = AgentAction(
        package='airbus_demo',
        executable='charge',
        name='charge'
    )

    # perform pickup
    pickup = AgentAction(
        package='airbus_demo',
        executable='pickup',
        name='pickup'
    )

    # perform drop
    drop = AgentAction(
        package='airbus_demo',
        executable='drop',
        name='drop'
    )

    # perform change_tool
    change_tool = AgentAction(
        package='airbus_demo',
        executable='change_tool',
        name='change_tool'
    )

    ld = AgentLaunchDescription(
        agent_id=AGENT_ID,
        agent_group=AGENT_GROUP_ID,
        init_params={
            'pddl_file': airbus_demo_share_dir + '/pddl/transporting-agent.pddl',
            'init_bset': airbus_demo_share_dir + '/launch/ta1/init_bset.yaml',
            'init_dset': airbus_demo_share_dir + '/launch/ta1/init_dset.yaml',
            'init_reactive_rules_set': airbus_demo_share_dir + '/launch/ta1/init_rrules.yaml',
            'debug_log_active': ['belief_manager', 'scheduler', 'plan_director']
        },
        actions=[moveto, charge, pickup, drop, change_tool],
        sensors=[],
        run_only_psys2=False
    ) 

    return ld