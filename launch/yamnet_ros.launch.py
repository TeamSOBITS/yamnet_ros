import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    pkg_share = get_package_share_directory('yamnet_ros')

    # ── Launch arguments ──────────────────────────────────────────────
    audio_device_arg = DeclareLaunchArgument(
        'audio_device',
        default_value='',
        description='ALSA capture device.  Empty = auto-detect first available device.  '
                    'Use arecord -l to list alternatives (e.g. hw:1,7, plughw:1,0).',
    )
    audio_channels_arg = DeclareLaunchArgument(
        'audio_channels',
        default_value='0',
        description='Number of capture channels (0 = auto: 2 for DMIC, 1 for analog mic).',
    )
    threshold_arg = DeclareLaunchArgument(
        'detection_threshold',
        default_value='0.15',
        description='YAMNet score threshold [0-1] to publish a detection.',
    )
    hop_secs_arg = DeclareLaunchArgument(
        'hop_secs',
        default_value='0.5',
        description='Inference interval in seconds (lower = faster response, more CPU).',
    )
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Node namespace.',
    )
    weights_arg = DeclareLaunchArgument(
        'weights_path',
        default_value=os.path.join(pkg_share, 'weights', 'yamnet.h5'),
        description='Absolute path to yamnet.h5 weights file.',
    )

    # ── Node ─────────────────────────────────────────────────────────
    yamnet_node = LifecycleNode(
        package='yamnet_ros',
        executable='yamnet_node',
        name='yamnet_ros',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'yamnet_ros.yaml'),
            {
                'audio_device':        LaunchConfiguration('audio_device'),
                'audio_channels':      LaunchConfiguration('audio_channels'),
                'detection_threshold': LaunchConfiguration('detection_threshold'),
                'hop_secs':            LaunchConfiguration('hop_secs'),
                'weights_path':        LaunchConfiguration('weights_path'),
            },
        ],
    )

    # ── Auto-configure on startup ─────────────────────────────────────
    # Node starts unconfigured; after 2 s it is automatically configured
    # (model loaded, inactive state). The task activates/deactivates as needed.
    configure_on_start = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/yamnet_ros', 'configure'],
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        audio_device_arg,
        audio_channels_arg,
        threshold_arg,
        hop_secs_arg,
        namespace_arg,
        weights_arg,
        yamnet_node,
        configure_on_start,
    ])
