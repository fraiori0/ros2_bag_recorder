#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.parameter import Parameter

import rosbag2_py
import datetime

from ros2_bag_recorder_interfaces.action import RecorderManager
import yaml

def nested_dict_update(destination, source):
    """
    Recursively updates a nested dictionary taking values from another nested dictionary.
    """
    for key, value in source.items():
        if isinstance(value, dict) and key in destination and isinstance(destination[key], dict):
            # If both values are dictionaries, recurse
            nested_dict_update(destination[key], value)
        else:
            # Otherwise, update the destination with the new value
            destination[key] = value
    
    return destination

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
        self.config = {
            "destination": {
                    "target_folder": "~/tmp_recordings/",
                    "filename_prefix": ""
                }
        }
        if config_path:
            # Load configuration from YAML if provided
            self.load_config(config_path)
        else:
            raise RuntimeError('A config_path parameter must be provided')
        
        self.recorder = None  
        self.recording = False
        
        # Action server
        self.server = ActionServer(
            self,
            RecorderManager,
            "recording_manager",
            self.execute_callback,
            goal_callback=self.goal_callback,
            # cancel_callback=self.cancel_callback
        )
        
        
        self.get_logger().info('Recording manager node initialized')
        self.get_logger().info(f'Current config: {self.config}')
    
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
        
        self.config = nested_dict_update(self.config, configs)
        
        os.makedirs(self.config["destination"]["target_folder"], exist_ok=True)
        
        # Update the configuration
        
        # this should support runtime change of the parameters, if we pass new ones and call update config
        self.storage_options = rosbag2_py.StorageOptions(
            uri = self.config["destination"]["target_folder"],
            storage_id = "sqlite3",
            # check here for what can be passed to this function
            # https://github.com/ros2/rosbag2/blob/6b462de40afa0cff207a5da6301e85cb108233ad/rosbag2_storage/include/rosbag2_storage/storage_options.hpp
            **self.config["storage"],
        )
        
        # check here for what can be passed to this function
        # https://github.com/ros2/rosbag2/blob/6b462de40afa0cff207a5da6301e85cb108233ad/rosbag2_py/rosbag2_py/_transport.pyi
        self.record_options = rosbag2_py.RecordOptions()
        # set values from config
        for key, value in self.config["record"].items():
            if hasattr(self.record_options, key):
                setattr(self.record_options, key, value)
            else:
                self.get_logger().warn(f"Unknown RecordOptions attribute: {key}")
    
    
    def goal_callback(self, goal_request):
        """Accept or reject goal requests."""
        if self.recording:
            self.get_logger().warn("Recording already in progress, rejecting goal")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT
    
    # def cancel_callback(self, goal_handle):
    #     """Cancellation requests."""
    #     self.get_logger().info('Received cancellation request')
    #     return CancelResponse.ACCEPT
    
    def execute_callback(self, goal_handle):
        
        # Set recording flag
        self.recording = True
        # if self.recording:
        #     self.get_logger().warn("A recording is already in progress, stop it before starting a new one")
        
        try:
            
            
            self.start_recording()
            
            success = self.monitor_recording(goal_handle)
            
            # Stop recording
            self.stop_recording()
            
            # get size
            size_mb = self.get_total_bag_size() / (1024 * 1024)
            
            result = RecorderManager.Result()
            if success:
                result.success = True
                result.message = f"Recording completed. Size: {size_mb:.2f} MB"
                goal_handle.succeed()
            else:
                result.success = False
                result.message = f"Recording cancelled. Size: {size_mb:.2f} MB"
                goal_handle.abort()
        
        except Exception as e:
            self.get_logger().error(f"Something failed while recording:\n\t\'{e}\'")
            self.stop_recording()# Create error result
            result = RecorderManager.Result()
            result.success = False
            result.message = f"Recording failed: {str(e)}"
            goal_handle.abort()
            
        finally:
            self.recording = False
        
        return result
    
    def start_recording(self):
        """Start the recorder."""
        # Update uri with a subdirectory containing the filename prefix and a timestamp
        bag_path = os.path.join(
            self.config["destination"]["target_folder"],
            f"{self.config['destination']['filename_prefix']}_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}"
        )
        self.storage_options.uri = bag_path
        
        # Initialize recorder
        self.recorder = rosbag2_py.Recorder(
            self.storage_options,
            self.record_options,
        )
        
        self.recorder.start_spin()
        self.recorder.record()
        
        self.get_logger().info('Recording started')
        self.get_logger().info(f'\trecording to: {bag_path}')
    
    def stop_recording(self):
        """Stop the current recording session (if any)."""
        if self.recorder is not None:
            try:
                self.recorder.stop()
                self.recorder.stop_spin()
                self.get_logger().info('Recording stopped')
            except Exception as e:
                self.get_logger().error(f'Error stopping recorder: {str(e)}')
            
            self.recorder = None
        
        
    def monitor_recording(self, goal_handle):
        """
        Monitor the recording process, checking for duration and size limits.
        Returns:
            bool: True if completed normally, False if cancelled
        """
        max_duration = goal_handle.request.max_duration  # seconds
        max_size_bytes = 1024 * 1024 * goal_handle.request.max_size  # MegaBytes
        
        # Get start time to measure duration of recording
        start_time  = self.get_clock().now().nanoseconds/1.0e9  # seconds
        
        check_interval = 0.5  # Check every <-- seconds
        
        while True:
            # while True:
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return False
            
            
            # Check duration limit
            elapsed_time = (self.get_clock().now().nanoseconds/1.0e9) - start_time
            if (max_duration > 0) and (elapsed_time >= max_duration):
                self.get_logger().info(f'Duration limit rfor recording reached ({max_duration}s)')
                return True
            
            # Check size limit
            current_size = self.get_total_bag_size()
            if current_size >= max_size_bytes:
                self.get_logger().info(f'Size limit reached ({current_size/(1024*1024):.2f}MB)')
                return True
            
            # Send feedback
            self.publish_feedback(goal_handle, elapsed_time, current_size)
            
            # Wait before next check
            rclpy.spin_once(self, timeout_sec=check_interval)
            
    def get_total_bag_size(self) -> int:
        """Get total size of all files in the bag directory."""
        # if not self._storage_options or not self._storage_options.uri:
        #     return 0.0
            
        bag_directory = self.storage_options.uri
        if not os.path.exists(bag_directory):
            return 0.0
        
        total_size = 0.0
        try:
            for filename in os.listdir(bag_directory):
                file_path = os.path.join(bag_directory, filename)
                if os.path.isfile(file_path):
                    total_size += os.path.getsize(file_path)
        except OSError:
            pass  # Ignore file system errors
            
        return total_size

    def publish_feedback(self, goal_handle, elapsed_time, size):
        """Send feedback to the action client."""
        try:
            feedback = RecorderManager.Feedback()
            feedback.elapsed_time = elapsed_time
            feedback.size_mb =  size / (1024 * 1024)
            goal_handle.publish_feedback(feedback)
            
        except Exception as e:
            self.get_logger().warn(f'Error publishing feedback: {str(e)}')
        
    
    def destruct(self):
        if self.recording:
            self.get_logger().info('Node shutting down, stopping recording...')
            self.stop_recording()
        
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