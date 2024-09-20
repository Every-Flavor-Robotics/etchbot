"""
config.py

This module defines a singleton Config class that loads configuration data from a YAML file.
It ensures that only one instance of the configuration is created and provides thread-safe access to the configuration data.

Usage example:
    config = Config('/path/to/config.yaml')
    value = config.get('some_key', 'default_value')
"""

import yaml
from threading import Lock

class Config:
    """
    Singleton class to manage configuration data loaded from a YAML file.
    """
    _instance = None
    _lock = Lock()

    def __new__(cls, yaml_file):
        """
        Create a new instance of Config if it doesn't exist, otherwise return the existing instance.
        This method is thread-safe.

        :param yaml_file: Path to the YAML configuration file.
        :return: Singleton instance of Config.
        """
        with cls._lock:
            if cls._instance is None:
                cls._instance = super(Config, cls).__new__(cls)
                cls._instance._initialize(yaml_file)
        return cls._instance

    def _initialize(self, yaml_file):
        """
        Initialize the Config instance by loading the YAML file.

        :param yaml_file: Path to the YAML configuration file.
        """
        with open(yaml_file, 'r') as file:
            self._config = yaml.safe_load(file)

    def get(self, key, default=None):
        """
        Retrieve a value from the configuration data, supporting nested keys.

        :param key: The key to look up in the configuration. Use dot notation for nested keys.
        :param default: The default value to return if the key is not found.
        :return: The value associated with the key, or the default value if the key is not found.
        """
        keys = key.split('.')
        value = self._config
        for k in keys:
            if isinstance(value, dict) and k in value:
                value = value[k]
            else:
                return default
        return value