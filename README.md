# Custom Nodes for ROS2

This workspace contains custom ROS2 nodes for various functionalities, including sensor data processing, computer vision, and communication.


## Dependencies

This package depends on the following ROS2 packages:
- `rclcpp`
- `sensor_msgs`
- `cv_bridge`
- `image_transport`
- `realsense2_camera`
- `ultralytics_ros`
- `optris_drivers2`
- `launch`
- `launch_ros`

## Usage

To launch the main application, run the following command:

```bash
ros2 launch custom_nodes main_launch.py
```

This will launch all the necessary nodes, including the camera drivers, YOLO object detection, and the custom processing nodes.

The main launch file (`main_launch.py`) accepts several arguments to customize the execution:

- `align_depth`: set to `true` to align the depth and color streams. Default: `false`.
- `pointcloud_enable`: set to `true` to enable point cloud generation. Default: `false`.
- `debug`: set to `true` to launch in debug mode. Default: `false`.
- `focus`: sets the focus of the Optris camera. Default: `70`.
- `yolo_model`: the YOLO model to be used for object detection. Default: `General_Plants_19_06-seg.pt`.
- `rgbd_resolution`: the resolution of the RGBD camera. Default: `640,480,30`.
- `homography_file`: the homography file to be used. Default: `1280`.
- `name_class`: the name of the class to be detected by YOLO. Default: `plant`.
- `number_class`: the number of the class to be detected by YOLO. Default: `0`.

For example, to launch with depth alignment and point cloud enabled, you can run:

```bash
ros2 launch custom_nodes main_launch.py align_depth:=true pointcloud_enable:=true
```

# custom_nodes — brief handover

This folder contains ROS2 processing and driver helper scripts for the project: camera drivers, YOLO processing, temperature / CWSI computation, area segmentation, biomass estimation and an MQTT bridge. Below you will find the quick reproduction steps, where to find the important scripts and how to run them.

## Quick reproduction: play a rosbag (example)

To reproduce a recorded run from a rosbag directory (example provided), run:

bash
```
ros2 bag play <path_to_your_rosbag>
```

In a new terminal, source the ROS2 workspace and run the main launch file:

```bash
source install/setup.bash
ros2 launch custom_nodes main_launch.py
```

This will replay the rosbag and launch the necessary nodes for processing.

## Scripts

The `scripts` directory contains several Python scripts that implement the core functionalities of this package.

- `ros2_mqtt_publisher.py`: This script subscribes to various ROS2 topics, collects data, and publishes it to an MQTT broker. It also logs the data to a CSV file.
- `area_segmentation_node.py`: This script performs area segmentation on images.
- `biomass_node.py`: This script calculates the biomass of plants.
- `crop_light_state_node.py`: This script determines the light state of the crops (sun or shade).
- `GPS_coordinates_node.py`: This script reads GPS coordinates and publishes them.
- `higrometer_node.py`: This script reads data from a hygrometer.
- `modify_xml.py`: This script modifies the configuration file for the Optris camera.
- `record_temperature.py`: This script records temperature data.
- `sun_shadow_calibration.py`: This script performs sun/shadow calibration.
- `temperature_cswi_calculation.py`: This script calculates the Crop Water Stress Index (CWSI).
- `utm_base_link_xy.py`: This script publishes the TF transform between the UTM and base_link frames.

Notes:
- The `--clock` flag publishes simulated /clock for nodes that use `use_sim_time`.
- Use the path above as-is or replace with your bag directory.

## Launching the workspace

The main launch file is the entry point for both simulated (rosbag playback) and real-hardware runs:

- To run in "rosbag / simulated time" mode (no hardware drivers; nodes use the bag's clock):
  - ros2 launch custom_nodes main_launch.py use_sim_time:=true
  - In example: `` ros2 bag play solarsemi_rows5-6_2026-02-11_11-49-06/ --topics /goal_pose /clicked_point /pce_p18/abs_humidity /initialpose /crop_light_state /tf_utm_baselink /thermal_image_view /tf_static /camera/camera/depth/camera_info                     /thermal_image /parameter_events /gps/fix /odometry/local /tf /rosout /pce_p18/dew_point /odometry/gps /Temperature_and_CSWI/masked_image_with_temperature /camera/camera/depth/image_rect_raw /controller/odometry /gps/                         filtered /NDVI /camera/camera/color/image_raw /pce_p18/temperature /pce_p18/rel_humidity /segmentation_area_info /odometry/global /Temperature_and_CSWI/text /Temperature_and_CSWI/rescaled_rgb --clock``


- To run with real hardware (drivers enabled; live sensors):
  - ros2 launch custom_nodes main_launch.py use_sim_time:=false

See the main launch file here:
- [launch/main_launch.py](launch/main_launch.py)

## Important scripts & nodes

Processing, drivers and utilities live in `scripts/`. Key files:

- MQTT bridge (collects many topics → MQTT + CSV): [`Ros2MqttPublisher`](scripts/ros2_mqtt_publisher.py) — [scripts/ros2_mqtt_publisher.py](scripts/ros2_mqtt_publisher.py)
- Temperature + CWSI processing: [`Calculator`](scripts/temperature_cswi_calculation.py) — [scripts/temperature_cswi_calculation.py](scripts/temperature_cswi_calculation.py)
- Biomass estimation node: [`BiomassNode`](scripts/biomass_node.py) — [scripts/biomass_node.py](scripts/biomass_node.py)
- Area segmentation processing: [`AreaSegmentNode`](scripts/area_segmentation_node.py) — [scripts/area_segmentation_node.py](scripts/area_segmentation_node.py)
- Rosbag helper / recorder: [`RosbagRecorder`](scripts/rosbag_recording.py) — [scripts/rosbag_recording.py](scripts/rosbag_recording.py)
- UTM → base_link TF publisher: [scripts/utm_base_link_xy.py](scripts/utm_base_link_xy.py)

Camera-related launch / helpers:
- Realsense launch wrapper: [launch/launch_realsense.py](launch/launch_realsense.py)
- Thermal camera (Optris) helper: [launch/launch_thermalcamera.py](launch/launch_thermalcamera.py)
- RViz config used in the system: [launch/rviz2_config4.rviz](launch/rviz2_config4.rviz)

GUI / Launcher utilities (local Tk GUI to start/stop processes):
- GUI app: [launch/APP.py](launch/APP.py)
- Generic Node launcher: [`NodeLauncher`](launch/nodos/NodeLauncher.py) — [launch/nodos/NodeLauncher.py](launch/nodos/NodeLauncher.py)
- Thermal options UI: [`ThermalCameraOptions`](launch/nodos/ThermalCameraOptions.py) — [launch/nodos/ThermalCameraOptions.py](launch/nodos/ThermalCameraOptions.py)
- Biomass options UI: [launch/nodos/BiomassOptions.py](launch/nodos/BiomassOptions.py)
- Area segmentation UI: [launch/nodos/AreaSegmentationOptions.py](launch/nodos/AreaSegmentationOptions.py)

Homography / calibration resources:
- Homography files used by the processing node: [Homography/average_homography_640.txt](Homography/average_homography_640.txt), [Homography/average_homography2.txt](Homography/average_homography2.txt), and config: [Homography/config.xml](Homography/config.xml)

Package metadata / build:
- [package.xml](package.xml)
- [CMakeLists.txt](CMakeLists.txt)

## Typical workflow to "replay and process"

1. Start bag playback (see the ros2 bag play command above).
2. Launch processing stack in simulated mode so nodes consume the bag `/clock`:
   - ros2 launch custom_nodes main_launch.py use_sim_time:=true
3. Optional: open RViz using the included config to inspect images and pointclouds:
   - RViz is launched automatically by the main launch; the file is [launch/rviz2_config4.rviz](launch/rviz2_config4.rviz).
4. Inspect logs and CSV output created by the MQTT bridge at `~/sensors_ws/data_collection` (see [`Ros2MqttPublisher`](scripts/ros2_mqtt_publisher.py)).

## Quick developer notes & gotchas

- The main processing node that computes temperature/CWSI expects homography matrices and will load them from `Homography/` depending on RGB resolution. See [`Calculator`](scripts/temperature_cswi_calculation.py) for details.
- The biomass estimator uses depth + color to build pointclouds (see [`BiomassNode.create_pointcloud`](scripts/biomass_node.py)).
- The GUI launcher spawns nodes inside embedded xterm widgets; logs are redirected to `/tmp/*.log` — see [`NodeLauncher`](launch/nodos/NodeLauncher.py) and [`ThermalCameraOptions`](launch/nodos/ThermalCameraOptions.py).
- If you play a bag and do not set `use_sim_time:=true` nodes will try to use live hardware and some drivers will be launched (optris, realsense, etc.). Use `use_sim_time:=true` for pure playback.

## Where to look for common tasks

- Add a new rosbag topic to be recorded / played: edit [scripts/rosbag_recording.py](scripts/rosbag_recording.py)
- Change CSV fields or add a new sensor mapping: edit [`Ros2MqttPublisher`](scripts/ros2_mqtt_publisher.py)
- Tune homography / calibration: [Homography/config.xml](Homography/config.xml) and homography matrix files in [Homography/](Homography/)
- GUI tweaks: [launch/APP.py](launch/APP.py) and the set of options under [launch/nodos/](launch/nodos/)

## Final notes

- This README is a pragmatic summary. For detailed behavior of a node, open its script in `scripts/` (examples linked above).
- Build/installation of the ROS package uses the top-level [CMakeLists.txt](CMakeLists.txt) and [package.xml](package.xml).

Good luck. If anything in these scripts needs a short docstring added inline, add it near the relevant class/function:
- [`BiomassNode`](scripts/biomass_node.py)
