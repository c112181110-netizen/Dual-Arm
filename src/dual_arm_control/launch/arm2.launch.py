from pathlib import Path
import hashlib
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    mode = value('mode')
    if mode not in ('demo', 'hardware'):
        raise ValueError('mode must be demo or hardware')
    demo = mode == 'demo'
    control = Path(get_package_share_directory('dual_arm_control'))
    description = Path(get_package_share_directory('dual_arm_description'))
    config_path = Path(value('config') or control / 'config' / ('arm2_demo.yaml' if demo else 'arm2_hardware.yaml'))
    config = yaml.safe_load(config_path.read_text())
    task = dict(config['arm2_coordinator']['ros__parameters'])
    bridge = dict(config['cm530_bridge']['ros__parameters'])
    task['simulation'], bridge['mock_hardware'] = demo, demo
    auto_confirm = value('auto_confirm_demo').lower() == 'true'
    if auto_confirm and not demo:
        raise ValueError('auto_confirm_demo cannot be enabled with hardware')
    task['auto_confirm_demo'] = auto_confirm
    if value('journal'):
        task['journal_path'] = value('journal')
    if not demo and (not task.get('calibrated') or not bridge.get('hardware_calibrated') or not bridge.get('serial_port')):
        raise ValueError('Hardware calibration and CM530 port are missing; edit a measured hardware config first')
    if not demo and (not value('robot_urdf') or not value('robot_srdf')):
        raise ValueError('Hardware requires explicitly measured robot_urdf and robot_srdf; demo model is not calibration')
    urdf = Path(value('robot_urdf') or description / 'urdf/arm1.urdf').read_text()
    srdf = Path(value('robot_srdf') or description / 'srdf/arm1.srdf').read_text()
    if demo and not value('robot_urdf'):
        urdf = urdf.replace('arm1', 'arm2')
    if demo and not value('robot_srdf'):
        srdf = srdf.replace('arm1', 'arm2')
    task['model_signature'] = hashlib.sha256((urdf + srdf).encode()).hexdigest()
    # Fail before starting ANY process if task calibration/geometry is invalid.
    from dual_arm_control.arm2_coordinator import ARM2_DEFAULTS
    from dual_arm_control.arm2_config import validate_arm2
    from dual_arm_control.coordinator import validate_config
    checked = {**ARM2_DEFAULTS, **task}
    validate_config(checked)
    validate_arm2(checked)
    bridge['arms'] = ['arm2']
    bridge['require_safety_inputs'] = True
    bridge['safety_max_age'] = task.get('safety_max_age', 2.0)
    robot = {'robot_description': ParameterValue(urdf, value_type=str)}
    joints = ['arm2_' + j for j in ('arm_shoulder_pan_joint', 'arm_shoulder_lift_joint',
                                   'arm_elbow_flex_joint', 'arm_wrist_flex_joint')]
    moveit = {**robot, 'robot_description_semantic': ParameterValue(srdf, value_type=str),
        'robot_description_kinematics': {'arm2': {
            'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
            'kinematics_solver_search_resolution': 0.005, 'kinematics_solver_timeout': 0.2}},
        'robot_description_planning': {'joint_limits': {j: {
            'has_velocity_limits': True, 'max_velocity': 0.52359878,
            'has_acceleration_limits': True, 'max_acceleration': 1.0} for j in joints}},
        'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl',
        'ompl': {
            'planning_plugins': ['ompl_interface/OMPLPlanner'],
            'request_adapters': ['default_planning_request_adapters/ResolveConstraintFrames',
                                 'default_planning_request_adapters/ValidateWorkspaceBounds',
                                 'default_planning_request_adapters/CheckStartStateBounds',
                                 'default_planning_request_adapters/CheckStartStateCollision'],
            'response_adapters': ['default_planning_response_adapters/AddTimeOptimalParameterization',
                                  'default_planning_response_adapters/ValidateSolution',
                                  'default_planning_response_adapters/DisplayMotionPath'],
            'planner_configs': {'RRTConnectkConfigDefault': {'type': 'geometric::RRTConnect', 'range': 0.0}},
            'arm2': {'planner_configs': ['RRTConnectkConfigDefault'], 'longest_valid_segment_fraction': 0.005}},
        'allow_trajectory_execution': True,
        'trajectory_execution.allowed_start_tolerance': 0.04,
        'trajectory_execution.allowed_execution_duration_scaling': 2.0,
        'trajectory_execution.allowed_goal_duration_margin': 15.0,
        'publish_robot_description_semantic': True,
        'planning_scene_monitor.publish_planning_scene': True,
        'planning_scene_monitor.publish_geometry_updates': True,
        'planning_scene_monitor.publish_state_updates': True,
        'planning_scene_monitor.publish_transforms_updates': True,
    }
    moveit.update(yaml.safe_load((control / 'config/moveit_controllers.yaml').read_text().replace('arm1', 'arm2')))
    nodes = [
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[robot], output='screen'),
        Node(package='dual_arm_control', executable='cm530_bridge', parameters=[bridge], output='screen'),
        Node(package='moveit_ros_move_group', executable='move_group', parameters=[moveit], output='screen'),
        Node(package='dual_arm_control', executable='arm2_coordinator', parameters=[task], output='screen'),
    ]
    if demo:
        mock_params = dict(config.get('arm2_mock_vision', {}).get('ros__parameters', {}))
        mock_params['c_poses'] = task['c_poses']
        nodes.append(Node(package='dual_arm_control', executable='arm2_mock_vision',
                          parameters=[mock_params], output='screen'))
    handlers = [RegisterEventHandler(OnProcessExit(target_action=n,
        on_exit=[EmitEvent(event=Shutdown(reason='A required process exited; no automatic restart'))])) for n in nodes]
    return nodes + handlers


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='demo'),
        DeclareLaunchArgument('config', default_value=''),
        DeclareLaunchArgument('auto_confirm_demo', default_value='false'),
        DeclareLaunchArgument('journal', default_value=''),
        DeclareLaunchArgument('robot_urdf', default_value=''),
        DeclareLaunchArgument('robot_srdf', default_value=''),
        OpaqueFunction(function=setup),
    ])
