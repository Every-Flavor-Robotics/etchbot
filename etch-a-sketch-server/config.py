import yaml
from threading import Lock

class Config:
    """
    Singleton class to manage configuration data loaded from a YAML file.
    """
    _instance = None
    _lock = Lock()

    def __new__(cls, yaml_file=None):
        """
        Create a new instance of Config if it doesn't exist, otherwise return the existing instance.
        This method is thread-safe.

        :param yaml_file: (Optional) Path to the YAML configuration file.
        :return: Singleton instance of Config.
        """
        with cls._lock:
            if cls._instance is None:
                if yaml_file is None:
                    raise ValueError("First initialization requires a yaml_file path.")
                cls._instance = super(Config, cls).__new__(cls)
                cls._instance._initialize(yaml_file)
            else:
                if yaml_file is not None and yaml_file != cls._instance.yaml_file:
                    raise ValueError(
                        f"Config is already initialized with a different yaml_file: {cls._instance.yaml_file}"
                    )
            return cls._instance

    def _initialize(self, yaml_file):
        """
        Initialize the Config instance by loading the YAML file.

        :param yaml_file: Path to the YAML configuration file.
        """
        self.yaml_file = yaml_file  # Store the yaml_file path
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