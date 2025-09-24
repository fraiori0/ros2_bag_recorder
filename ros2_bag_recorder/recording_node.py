#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.parameter import Parameter

import rosbag2_py

from ros2_bag_recorder_interfaces.action import RecorderManager
from .recording_manager import ConfigManager
import yaml



class RecordingNode(Node):
    """
    ROS2 Node instantiating an Action Sever that allows to start/stop recordings of rosbag, and to monitor recording.
    
    Basic options (topics, splitting size, compression, etc.) are configured via a YAML
        file, whose path should be passed as parameter to the node.
    
    A default config file is contained in this package at config/default_config.yaml;
        the path to this file is given as default to this node if using the launch script in this package.
    """
    
    def __init__(self):
        super().__init__('recording_manager')

        # Declare parameters
        self.declare_parameter('config_path', '')
        
        # Get config file path
        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        
        # Initialize recording manager
        self.config_recording = {}
        self.config_storage = {}
        
        if config_path:
            # Load configuration from YAML if provided
            self.load_config(config_path)
        else:
            raise RuntimeError('A config_path parameter must be provided')
        
        # Simple switch state to track if we are recording or not    
        self.recording = False
        
        # Action server
        self.server = ActionServer(
            self,
            RecorderManager,
            "recording_manager",
            self.server_callback(),
        )
        
        
        self.get_logger().info('Recording manager node initialized')
        self.get_logger().info(f'Current config: {self.manager.config}')
    
    def load_config(self, config_path: str) -> None:
        """
        Load configuration from YAML file.
        
        Args:
            config_path: Path to YAML configuration file
        """
        with open(config_path, 'r') as file:
            yaml_config = yaml.safe_load(file)               
            if yaml_config:
                self.update_config(yaml_config)
    
    def update_config(self, configs) -> None:
        """
        Update configuration with new values (e.g., from launch parameters).
        
        Args:
            configs: Dictionary of configuration updates
        """
        
        # NOTE: we update with empty dictionary if some configs are missing
        if not ("storage" in configs.keys()):
            self.get_logger().warn("No \'storage\' key in the dictionary of configuration, using default values for storage options")
        if not ("recording" in configs.keys()):
            self.get_logger().warn("No \'recording\' key in the dictionary of configuration, using default values for recording options")
        
        self.config_storage.update(configs.get("storage", {}))
        self.config_recording.update(configs.get("recording", {}))
        
        # Update the configuration instances according to the configuration
        # this should support runtime change of the parameters, if we pass new ones and call update config
        self.storage_options = rosbag2_py.StorageOptions(
            # check here for what can be passed to this function
            # https://github.com/ros2/rosbag2/blob/6b462de40afa0cff207a5da6301e85cb108233ad/rosbag2_storage/include/rosbag2_storage/storage_options.hpp
            **self.config_storage,
        )
        self.record_options = rosbag2_py.RecordOptions(
            # check here for what can be passed to this function
            # https://github.com/ros2/rosbag2/blob/6b462de40afa0cff207a5da6301e85cb108233ad/rosbag2_py/rosbag2_py/_transport.pyi
            **self.config_record
        )
        
    def server_callback(self, goal_handle):
        
        if self.recording:
            self.get_logger().warn("A recording is already in progress, stop it before starting a new one")
        
        try:
            self.get_logger().info(f"Starting recording")
            
            self.recorder = rosbag2_py.Recorder(
                self.storage_options,
                self.record_options,
            )
        
        except Exception as e:
            self.get_logger().error(f"Failed to record: {e}")
            self.stop_recording
        
    
    def destruct(self):
        if self.manager.recording:
            self.get_logger().info('Node shutting down, stopping recording...')
            # SHUT DOWN RECORDING GRACEFULLY HERE
        
        super().destroy_node()


def main(args=None):
    """
    Entry point for the recording node.
    """
    rclpy.init(args=args)
    
    try:
        node = RecordingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()