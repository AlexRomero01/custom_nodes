"""Launch all nodes needed to use thermal, rgb cameras and biomass estimation"""
import os
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
    OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import xml.etree.ElementTree as ET


def generate_launch_description():

    # --- Arguments ---
    align_depth_launch_arg = DeclareLaunchArgument("align_depth", default_value='false')
    pointcloud_enable_launch_arg = DeclareLaunchArgument("pointcloud_enable", default_value='false')
    debug_launch_arg = DeclareLaunchArgument("debug", default_value='false')
    focus_launch_arg = DeclareLaunchArgument("focus", default_value='70')
    yolo_model_launch_arg = DeclareLaunchArgument("yolo_model", default_value='General_Plants_19_06-seg.pt')
    rgbd_resolution_launch_arg = DeclareLaunchArgument("rgbd_resolution", default_value='640,480,30')
    homography_file_launch_arg = DeclareLaunchArgument("homography_file", default_value='1280')
    name_class_id_launch_arg = DeclareLaunchArgument("name_class", default_value='plant')
    number_class_id_launch_arg = DeclareLaunchArgument("number_class", default_value='0')

    # --- Realsense Camera ---
    launch_include_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'align_depth.enable': LaunchConfiguration('align_depth'),
            'pointcloud.enable': LaunchConfiguration('pointcloud_enable'),
            'rgb_camera.color_profile': LaunchConfiguration('rgbd_resolution'),
            'depth_module.depth_profile': LaunchConfiguration('rgbd_resolution')
        }.items()
    )
    delay_before_launches = TimerAction(period=0.5, actions=[launch_include_1])

    # --- YOLO Tracker ---
    launch_include_2 = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ultralytics_ros'),
                'launch',
                'tracker.launch.xml'
            ])
        ]),
        launch_arguments={
            'debug': LaunchConfiguration('debug'),
            'yolo_model': LaunchConfiguration('yolo_model')
        }.items()
    )
    delay_between_launches = TimerAction(period=3.0, actions=[launch_include_2])

    # --- Optris Camera Config ---
    focus_value = LaunchConfiguration('focus')
    config_file_path = os.path.expanduser('~/sensors_ws/src/custom_nodes/Homography/config.xml')
    modify_xml_file_path = os.path.expanduser('~/sensors_ws/src/custom_nodes/scripts/modify_xml.py')

    modify_xml_process = ExecuteProcess(
        cmd=['python3', modify_xml_file_path, config_file_path, focus_value],
        output='screen'
    )
    delay_config = TimerAction(period=0.5, actions=[modify_xml_process])

    optris_imager_node = ExecuteProcess(
        cmd=['xterm', '-e', 'ros2', 'run', 'optris_drivers2', 'optris_imager_node', config_file_path],
        shell=True
    )
    delay_node_1 = TimerAction(period=1.5, actions=[optris_imager_node])

    optris_colorconvert_node = ExecuteProcess(
        cmd=['xterm', '-e', 'ros2', 'run', 'optris_drivers2', 'optris_colorconvert_node'],
        shell=True
    )
    delay_node_2 = TimerAction(period=2.5, actions=[optris_colorconvert_node])

    # --- Temperature + CWSI ---
    script_path = os.path.expanduser('~/sensors_ws/src/custom_nodes/scripts/temperature_cswi_calculation.py')
    mean_processor2_node = ExecuteProcess(
        cmd=[
            'xterm', '-e', 'python3', script_path,
            '--homography_file', LaunchConfiguration('homography_file'),
            '--name_class', LaunchConfiguration('name_class'),
            '--number_class', LaunchConfiguration('number_class')
        ],
        output='screen',
        shell=True
    )
    delay_py_node_2 = TimerAction(period=6.0, actions=[mean_processor2_node])

    # --- RViz ---
    rviz_config_file = os.path.expanduser('~/sensors_ws/src/custom_nodes/launch/rviz2_config4.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    delay_rviz = TimerAction(period=9.0, actions=[rviz_node])

    # --- GPS Node ---
    GPS_coordinates_node = ExecuteProcess(
        cmd=['xterm', '-e', 'python3', os.path.expanduser('~/sensors_ws/src/custom_nodes/scripts/GPS_coordinates_node.py')],
        output='screen',
        shell=True
    )
    delay_gps_node = TimerAction(period=6.0, actions=[GPS_coordinates_node])

    # --- Area Segmentation Node ---
    area_segment_node = ExecuteProcess(
        cmd=['xterm', '-e', 'python3', os.path.expanduser('~/sensors_ws/src/custom_nodes/scripts/area_segmentation_node.py')],
        output='screen',
        shell=True
    )
    delay_area_node = TimerAction(period=6.0, actions=[area_segment_node])

    # --- Crop Light State Node ---
    crop_light_state_node = ExecuteProcess(
        cmd=['xterm', '-e', 'python3', os.path.expanduser('~/sensors_ws/src/custom_nodes/scripts/crop_light_state_node.py')],
        output='screen',
        shell=True
    )
    delay_crop_light_state_node = TimerAction(period=6.0, actions=[crop_light_state_node])

    # --- Biomass Node ---
    biomass_node = ExecuteProcess(
        cmd=[
            'xterm', '-e', 'python3',
            os.path.expanduser('~/sensors_ws/src/custom_nodes/scripts/biomass_node.py')
        ],
        output='screen',
        shell=True
    )
    delay_biomass_node = TimerAction(period=6.0, actions=[biomass_node])

    # --- Higrometer Node ---
    higrometer_node = ExecuteProcess(
        cmd=[
            'xterm', '-e', 'python3',
            os.path.expanduser('~/sensors_ws/src/custom_nodes/scripts/higrometer_node.py')
        ],
        output='screen',
        shell=True
    )
    delay_higrometer_node = TimerAction(period=6.0, actions=[higrometer_node])

    # --- NDVI Node ---
    ndvi_node_process = ExecuteProcess(
        cmd=['xterm', '-e', 'python3', '~/sensors_ws/src/ndvi_sensor/scripts/ndvi_sensor_node.py'],
        output='screen',
        shell=True
    )
    delay_ndvi_node = TimerAction(period=6.0, actions=[ndvi_node_process])

    # --- MQTT Publisher Node ---
    mqtt_node_process = ExecuteProcess(
        cmd=['xterm', '-e', 'python3', '~/sensors_ws/src/custom_nodes/scripts/ros2_mqtt_publisher.py'],
        output='screen',
        shell=True
    )
    delay_mqtt_node = TimerAction(period=6.0, actions=[mqtt_node_process])


    # --- UTM → base_link TF Publisher ---
    utm_base_link_node = ExecuteProcess(
        cmd=['xterm', '-e', 'python3', '~/sensors_ws/src/custom_nodes/scripts/utm_base_link_xy.py'],
        output='screen',
        shell=True
    )
    delay_utm_node = TimerAction(period=6.0, actions=[utm_base_link_node])


    # --- Return Launch Description ---
    return LaunchDescription([
        align_depth_launch_arg,
        pointcloud_enable_launch_arg,
        debug_launch_arg,
        focus_launch_arg,
        yolo_model_launch_arg,
        rgbd_resolution_launch_arg,
        homography_file_launch_arg,
        name_class_id_launch_arg,
        number_class_id_launch_arg,
        delay_before_launches,
        delay_between_launches,
        delay_config,
        delay_node_1,
        delay_node_2,
        delay_py_node_2,
        delay_rviz,
        delay_gps_node,
        delay_area_node,
        delay_ndvi_node,
        delay_crop_light_state_node,
        delay_biomass_node,
        delay_higrometer_node,
        delay_mqtt_node,
        delay_utm_node
    ])
