import launch
import launch_ros
import launch.actions
import launch.substitutions
import launch_ros.actions
import yaml


def load_driver_config(path):
    if not path:
        return {}, {}, {}
    with open(path, 'r', encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file) or {}

    bringup_config = config.get('bringup', {}) or {}
    driver_launch_config = bringup_config.get('driver', {}) or {}

    if 'naoqi_driver' in config:
        driver_params = (config.get('naoqi_driver', {}) or {}).get('ros__parameters', {}) or {}
    else:
        driver_params = (config.get('/**', {}) or {}).get('ros__parameters', {}) or {}

    return bringup_config, driver_launch_config, driver_params


def launch_value(context, name):
    return launch.substitutions.LaunchConfiguration(name).perform(context)


def configured_value(context, name, config, default=''):
    value = launch_value(context, name)
    if value != '':
        return value
    return config.get(name, default)


def launch_setup(context, *args, **kwargs):
    driver_params_file = launch.substitutions.LaunchConfiguration('driver_params_file').perform(context)
    bringup_config, driver_launch_config, driver_params = load_driver_config(driver_params_file)
    command_topics = driver_launch_config.get('command_topics', {}) or {}
    parameters = [driver_params]

    node_params = {
        'nao_ip': configured_value(context, 'nao_ip', driver_params, '127.0.0.1'),
        'user': launch_value(context, 'username') or driver_params.get('user', 'nao'),
        'password': configured_value(context, 'password', driver_params, 'no_password'),
        'network_interface': configured_value(context, 'network_interface', driver_params, 'eth0'),
        'qi_listen_url': configured_value(context, 'qi_listen_url', driver_params, 'tcp://0.0.0.0:0'),
    }

    nao_port = configured_value(context, 'nao_port', driver_params, 9559)
    node_params['nao_port'] = int(nao_port)
    parameters.append(node_params)

    namespace = configured_value(context, 'namespace', bringup_config, '')
    boot_config_file = configured_value(context, 'boot_config_file', driver_launch_config, 'boot_config_NAO.json')
    cmd_vel_topic = configured_value(context, 'cmd_vel_topic', command_topics, 'cmd_vel')
    joint_trajectory_topic = configured_value(context, 'joint_trajectory_topic', command_topics, 'joint_trajectory')
    goal_pose_topic = configured_value(context, 'goal_pose_topic', command_topics, 'goal_pose')
    speech_topic = configured_value(context, 'speech_topic', command_topics, 'speech')

    return [
        launch.actions.SetEnvironmentVariable('NAOQI_DRIVER_BOOT_CONFIG_FILE', boot_config_file),
        launch_ros.actions.Node(
            package='naoqi_driver',
            executable='naoqi_driver_node',
            namespace=namespace,
            parameters=parameters,
            remappings=[
                ('cmd_vel', cmd_vel_topic),
                ('joint_trajectory', joint_trajectory_topic),
                ('goal_pose', goal_pose_topic),
                ('speech', speech_topic),
            ],
            output="screen")
    ]


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'boot_config_file',
            default_value="",
            description='Boot config preset from the naoqi_driver share directory'),
        launch.actions.DeclareLaunchArgument(
            'driver_params_file',
            default_value="",
            description='Optional bringup YAML file with launch settings and naoqi_driver parameters'),
        launch.actions.DeclareLaunchArgument(
            'nao_ip',
            default_value="",
            description='Ip address of the robot'),
        launch.actions.DeclareLaunchArgument(
            'nao_port',
            default_value="",
            description='Port to be used for the connection'),
        launch.actions.DeclareLaunchArgument(
            'username',
            default_value="",
            description='Username for the connection'),
        launch.actions.DeclareLaunchArgument(
            'password',
            default_value="",
            description='Password for the connection'),
        launch.actions.DeclareLaunchArgument(
            'network_interface',
            default_value="",
            description='Network interface to be used'),
        launch.actions.DeclareLaunchArgument(
            'qi_listen_url',
            default_value="",
            description='Endpoint to listen for incoming NAOqi connections (for audio)'),
        launch.actions.DeclareLaunchArgument(
            'namespace',
            default_value="",
            description='Name of the namespace to be used'),
        launch.actions.DeclareLaunchArgument('cmd_vel_topic', default_value=''),
        launch.actions.DeclareLaunchArgument('joint_trajectory_topic', default_value=''),
        launch.actions.DeclareLaunchArgument('goal_pose_topic', default_value=''),
        launch.actions.DeclareLaunchArgument('speech_topic', default_value=''),
        launch.actions.OpaqueFunction(function=launch_setup),
    ])