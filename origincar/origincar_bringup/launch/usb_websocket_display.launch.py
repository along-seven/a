import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory, get_package_prefix

def generate_launch_description():
    # 复制配置文件
    dnn_node_example_path = os.path.join(get_package_prefix('dnn_node_example'), "lib/dnn_node_example")
    os.system(f"cp -r {dnn_node_example_path}/config .")

    # 声明启动参数
    launch_args = [
        DeclareLaunchArgument("dnn_example_config_file", default_value=TextSubstitution(text="/root/dev_ws/config/yolov5xworkconfig.json")),
        DeclareLaunchArgument("dnn_example_dump_render_img", default_value=TextSubstitution(text="0")),
        DeclareLaunchArgument("dnn_example_image_width", default_value=TextSubstitution(text="480")),
        DeclareLaunchArgument("dnn_example_image_height", default_value=TextSubstitution(text="272")),
        DeclareLaunchArgument("dnn_example_msg_pub_topic_name", default_value=TextSubstitution(text="hobot_dnn_detection")),
        DeclareLaunchArgument('device', default_value='/dev/video0', description='usb camera device'),
    ]

    # 包含其他launch文件
    usb_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('hobot_usb_cam') + '/launch/hobot_usb_cam.launch.py'
        ),
        launch_arguments={
            'usb_image_width': '480', 
            'usb_image_height': '272',
            'usb_video_device': LaunchConfiguration('device')
        }.items()
    )

    nv12_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('hobot_codec') + '/launch/hobot_codec_decode.launch.py'
        ),
        launch_arguments={
            'codec_in_mode': 'ros', 
            'codec_out_mode': 'shared_mem',
            'codec_sub_topic': '/image', 
            'codec_pub_topic': '/hbmem_img'
        }.items()
    )

    jpeg_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('hobot_codec') + '/launch/hobot_codec_encode.launch.py'
        ),
        launch_arguments={
            'codec_in_mode': 'shared_mem', 
            'codec_out_mode': 'ros',
            'codec_sub_topic': '/hbmem_img', 
            'codec_pub_topic': '/image'
        }.items()
    )

    # 算法节点
    dnn_node_example_node = Node(
        package='dnn_node_example',
        executable='example',
        output='screen',
        parameters=[
            {"config_file": LaunchConfiguration('dnn_example_config_file')},
            {"dump_render_img": LaunchConfiguration('dnn_example_dump_render_img')},
            {"feed_type": 1},
            {"is_shared_mem_sub": 1},
            {"msg_pub_topic_name": LaunchConfiguration("dnn_example_msg_pub_topic_name")}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )
    
    image_transport_node = Node(
        package='utils',
        executable='image_transport_node',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    # 移除了web_node相关配置，不再启动WebSocket节点
    return LaunchDescription(launch_args + [
        usb_node,
        nv12_codec_node,
        jpeg_codec_node,  # 保留编解码节点以维持完整的数据处理流程
        dnn_node_example_node,
        image_transport_node
    ])
