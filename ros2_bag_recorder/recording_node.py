#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from .recording_manager import RecordingManager
from typing import Dict, Any
# NOte, this require the package with the custom message


class RecordingNode(Node):
    """
    ROS2 Node class that provides the interface for rosbag recording services.
    Uses RecordingManager for core functionality.
    """
    
    def __init__(self):
        super().__init__('recording_manager')

        
        # Declare parameters with defaults
        self.declare_parameter('config_path', '')
        
        # Get config file path
        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        
        # Initialize recording manager
        if config_path:
            self.get_logger().info(f'\tLoading config from: {config_path}')
            self.manager = RecordingManager(config_path)
        else:
            raise RuntimeError('A config_path parameter must be provided')
    
        
        # Create services
        self.start_service = self.create_service(
            Trigger,
            'recording_manager/start_recording',
            self.start_recording_callback
        )
        
        self.stop_service = self.create_service(
            Trigger,
            'recording_manager/stop_recording',
            self.stop_recording_callback
        )
        
        self.get_logger().info('Recording manager node initialized')
        self.get_logger().info(f'Current config: {self.manager.config}')
    
    def start_recording_callback(self, request, response):
        """
        Service callback to start recording.
        
        Args:
            request: Trigger service request
            response: Trigger service response
            
        Returns:
            Modified response with success status and message
        """
        success, message = self.manager.start_recording()
        response.success = success
        response.message = message
        
        if success:
            self.get_logger().info(f'Recording started: {message}')
        else:
            self.get_logger().warn(f'Failed to start recording: {message}')
        
        return response
    
    def stop_recording_callback(self, request, response):
        """
        Service callback to stop recording.
        
        Args:
            request: Trigger service request
            response: Trigger service response
            
        Returns:
            Modified response with success status and message
        """
        success, message = self.manager.stop_recording()
        response.success = success
        response.message = message
        
        if success:
            self.get_logger().info(f'Recording stopped: {message}')
        else:
            self.get_logger().warn(f'Failed to stop recording: {message}')
        
        return response
    
    def destroy_node(self):
        if self.manager.recording:
            self.get_logger().info('Node shutting down, stopping recording...')
            self.manager.stop_recording()
        
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