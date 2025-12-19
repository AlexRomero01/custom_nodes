#!/usr/bin/env python3
import subprocess
import signal
import os
import time

class RosbagRecorder:
    def __init__(self, topics, output_dir="rosbags", bag_name="my_bag"):
        self.topics = topics
        self.output_dir = os.path.expanduser(output_dir)
        os.makedirs(self.output_dir, exist_ok=True)

        timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
        self.bag_path = os.path.join(self.output_dir, f"{bag_name}_{timestamp}")

        self.process = None

    def start(self):
        """Start ros2 bag record with selected topics."""
        if self.process is not None:
            print("Rosbag already recording!")
            return

        cmd = ["ros2", "bag", "record", "-o", self.bag_path] + self.topics
        print(f"Starting rosbag: {' '.join(cmd)}")
        self.process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    def stop(self):
        """Stop recording by sending SIGINT (like Ctrl+C)."""
        if self.process is None:
            print("No rosbag recording active.")
            return

        print("Stopping rosbag...")
        self.process.send_signal(signal.SIGINT)
        self.process.wait()
        self.process = None
        print(f"Rosbag saved at {self.bag_path}")

if __name__ == "__main__":
    # Ask user for bag name
    bag_name = input("Enter a name for the rosbag file: ").strip()
    if not bag_name:
        bag_name = "plants"

    topics_to_record = [
        '/Temperature_and_CSWI/masked_image_with_temperature',
        '/Temperature_and_CSWI/rescaled_rgb',
        '/Temperature_and_CSWI/rescaled_yolo_masks',
        '/Temperature_and_CSWI/text',
        '/camera/camera/color/image_raw',
        '/camera/camera/depth/camera_info',
        '/camera/camera/depth/image_rect_raw',
        '/clicked_point',
        '/crop_light_state',
        '/crop_type',
        '/goal_pose',
        '/initialpose',
        '/parameter_events',
        '/plant_biomass',
        '/rosout',
        '/segmentation_area_info',
        '/tf',
        '/tf_static',
        '/thermal_image',
        '/thermal_image_view',
        '/visible_image',
        '/yolo_image',
        '/yolo_results',
        'pce_p18/temperature',
        'pce_p18/rel_humidity',
        'pce_p18/dew_point',
        'pce_p18/abs_humidity'
    ]

    recorder = RosbagRecorder(topics=topics_to_record, bag_name=bag_name)
    recorder.start()

    try:
        print("Recording... Press Ctrl+C to stop.")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        recorder.stop()
