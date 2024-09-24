# Script to run at build time to get the config for the config file

import yaml

Import("projenv")
env = DefaultEnvironment()

config_path = "../etchbot_config.yml"

# Print in green text
print("\033[92m" + "Configuring environment variables from config!" + "\033[0m")

# Create a list of keys to load, they should be hierarchical
keys = [
    "server.port",
    "server.host",
    "server.stream_port",
    "drawing.width",
    "drawing.height",
    "drawing.acceleration",
    "drawing.origin_x",
    "drawing.origin_y",
    "robot.gear_ratio",
    "robot.mm_to_knob_rad",
    "robot.left_right_backlash_mm",
    "robot.up_down_backlash_mm",
    "robot.backlash_compensation_radpersec",
    "robot.backlash_acceleration",
    "robot.max_acceleration",
    "robot.error_tolerance_mm"
]

# Load the config file
with open(config_path) as f:
    config = yaml.safe_load(f)

# Print in green, found config file
print("\033[92m" + f"Found config file at {config_path}" + "\033[0m")

print(keys)
for key in keys:
    # Split the key by dots
    key_parts = key.split(".")



    # Get the value from the config file
    value = config
    for part in key_parts:
        value = value[part]

    env_key = key_parts[-1].upper()


    # If value is a string, add it as a string
    if(isinstance(value, str)):
        value = f'\\"{value}\\"'

    print(f"\tSetting {env_key} to {value}")



    projenv.Append(CPPDEFINES=[(env_key, value)])
    env.Append(CPPDEFINES=[(env_key, value)])

    # env[env_key] = value

# Print in green, done setting environment variables
print("\033[92m" + "Done setting environment variables!" + "\033[0m")