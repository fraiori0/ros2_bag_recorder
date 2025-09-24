#!/usr/bin/env python3

import os
import yaml
import subprocess
import signal
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Any


class RecordingManager:
    """
    Manager class that handles the core functionality of rosbag recording.
    Manages configuration, file paths, and subprocess control for recording.
    """
    
    def __init__(self, config_path):
        """
        Initialize the RecordingManager with configuration.
        
        Args:
            config_path: Path to YAML config file
        """
        self.recording = False
        self.process = None
        
        self.config = {}
        
        # # Example configuration (also for debugging)
        # self.config = {
        #     'recorded_topics': [],
        #     'target_folder': '/tmp/rosbag_recordings',
        #     'filename_prefix': '',
        #     'split_size': 256,  # MB
        #     'compression_format': 'zstd', # or 'none'
        #     'compression_mode': 'file',  # 'file' or 'message'
        #     'max_bag_size': 0,   # 0 = unlimited
        #     'max_bag_duration': 0,  # 0 = unlimited (seconds)
        #     'publish': True,     # Republish topics while recording
        # }
        
        # Load configuration from YAML if provided
        if config_path:
            self.load_config(config_path)
    
    def load_config(self, config_path: str) -> None:
        """
        Load configuration from YAML file.
        
        Args:
            config_path: Path to YAML configuration file
        """
        with open(config_path, 'r') as file:
            yaml_config = yaml.safe_load(file)
            if yaml_config:
                self.config.update(yaml_config)
    
    def update_config(self, updates: Dict[str, Any]) -> None:
        """
        Update configuration with new values (e.g., from launch parameters).
        
        Args:
            updates: Dictionary of configuration updates
        """
        self.config.update(updates)
    
    def start_recording(self) -> tuple[bool, str]:
        """
        Start rosbag recording.
        
        Returns:
            Tuple of (success, message)
        """
        if self.recording:
            return False, 'A recording is already in progress, cannot start a new one'
        
        # Create target folder if it doesn't exist
        folder_path = self.config['target_folder']
        os.makedirs(folder_path, exist_ok=True)
        
        # Generate filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_name  = f"{self.config['filename_prefix']}_{timestamp}"
        
        # Build command for ros2 bag record
        cmd = self._build_record_command(bag_name)
        
        try:
            # Start subprocess
            self.process = subprocess.Popen(
                cmd,
                shell=True,
                start_new_session=True,
            )
            self.recording = True
            return True, f'Started recording to {folder_path}'
        except Exception as e:
            return False, f'Failed to start recording: {str(e)}'
    
    def stop_recording(self) -> tuple[bool, str]:
        """
        Stop rosbag recording.
        
        Returns:
            Tuple of (success, message)
        """
        if not self.recording:
            return False, 'No recording was in progress, nothing stopped'
        
        if self.process:
            try:
                # Terminate the subprocess group
                os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
                self.process.wait(timeout=10)
                self.process = None
                self.recording = False
            except Exception as e:
                return False, f'Failed to stop recording: {str(e)}'
        else:
            raise RuntimeError('Recording process is None while recording is True')
        
        return True, 'Stopped recording'
    
    def _build_record_command(self, bag_name: str) -> str:
        """
        Build the ros2 bag record command string.
        
        Args:
            bag_name: Name of the bag file (will be the name of the folder in which a database is save, in ROS 2)
            
        Returns:
            Command string for subprocess
        """
        bag_path = os.path.join(
            self.config['target_folder'],
            bag_name,
        )
        
        cmd_parts = ['ros2', 'bag', 'record']
        
        # Output name
        cmd_parts.extend(['-o', bag_path])
        
        # Compression
        if self.config['compression_format']:
            cmd_parts.extend(['--compression-format', self.config['compression_format']])
        if self.config['compression_mode']:
            cmd_parts.extend(['--compression-mode', self.config['compression_mode']])
        
        # Max bag size (in bytes)
        if self.config['max_bag_size'] > 0:
            max_size_bytes = int(self.config['max_bag_size'] * 1024 * 1024 * 1024)  # GB to bytes
            cmd_parts.extend(['--max-bag-size', str(max_size_bytes)])
        
        # Max duration (in seconds)
        if self.config['max_bag_duration'] > 0:
            cmd_parts.extend(['--max-bag-duration', str(self.config['max_bag_duration'])])
        
        # Add topics
        cmd_parts.extend(["--topics"] + list(self.config['recorded_topics']))
        
        return ' '.join(cmd_parts)
    
    def get_status(self) -> Dict[str, Any]:
        """
        Get current recording status and configuration.
        
        Returns:
            Dictionary with status information
        """
        return {
            'recording': self.recording,
            'config': self.config.copy(),
            'bag_process': self.process.pid if self.process else None,
        }