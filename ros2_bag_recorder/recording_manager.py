#!/usr/bin/env python3

import yaml

class ConfigManager:
    """
    Manager class that handles loading configuration.
    """
    
    def __init__(self, config_path):
        """
        Initialize the RecordingManager with configuration.
        
        Args:
            config_path: Path to YAML config file
        """
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
    
    def update_config(self, updates) -> None:
        """
        Update configuration with new values (e.g., from launch parameters).
        
        Args:
            updates: Dictionary of configuration updates
        """
        self.config.update(updates)
    