import transitions
import time
import threading
from pathlib import Path
from config import Config


class EtchBot:
    def __init__(self, ip, name):
        # IP address of the robot
        self.ip = ip
        # Name of the robot
        self.name = name

        # Robot states:
        # - WAITING_FOR_CONNECTION: The robot is not connected, but we are ready to accept a connection
        # - READY: The robot is connected and ready to draw
        # - DRAWING: The robot is drawing
        # - POST_DRAWING_ACTIONS: The robot has finished drawing, ready to erase
        # - ERASING: The robot is erasing
        # - POST_ERASING_ACTIONS: The robot has finished erasing
        # - REBOOTING: The robot is rebooting
        # - ERROR: The robot has encountered an error that requires manual intervention

        self.machine = transitions.Machine(
            model=self,
            states=[
                "WAITING_FOR_CONNECTION",
                "READY",
                "DRAWING",
                "POST_DRAWING_ACTIONS",
                "ERASING",
                "POST_ERASING_ACTIONS",
                "ERROR",
            ],
            initial="WAITING_FOR_CONNECTION",
            send_event=True,
            transitions=[
                {
                    "trigger": "connect",
                    "source": "WAITING_FOR_CONNECTION",
                    "dest": "READY",
                },
                {
                    "trigger": "connect",
                    "source": "READY",
                    "dest": "READY",
                },
                {
                    "trigger": "start_command",
                    "source": "READY",
                    "dest": "DRAWING",
                    "conditions": ["ready_to_draw", "cooldown_complete"],
                },
                {
                    "trigger": "start_command",
                    "source": "READY",
                    "dest": "ERASING",
                    "conditions": ["ready_to_erase"],
                },
                {
                    "trigger": "drawing_complete",
                    "source": "DRAWING",
                    "dest": "POST_DRAWING_ACTIONS",
                },
                {
                    "trigger": "post_drawing_actions_complete",
                    "source": "POST_DRAWING_ACTIONS",
                    "dest": "WAITING_FOR_CONNECTION",
                },
                {
                    "trigger": "erasing_complete",
                    "source": "ERASING",
                    "dest": "POST_ERASING_ACTIONS",
                },
                {
                    "trigger": "post_erasing_actions_complete",
                    "source": "POST_ERASING_ACTIONS",
                    "dest": "WAITING_FOR_CONNECTION",
                },
                {"trigger": "error", "source": "*", "dest": "ERROR"},
                {
                    "trigger": "timeout",
                    "source": [
                        "DRAWING",
                        "POST_DRAWING_ACTIONS",
                        "ERASING",
                        "POST_ERASING_ACTIONS",
                    ],
                    "dest": "ERROR",
                },
                {
                    "trigger": "timeout",
                    "source": ["READY"],
                    "dest": "WAITING_FOR_CONNECTION",
                },
                {
                    "trigger": "recover",
                    "source": "ERROR",
                    "dest": "WAITING_FOR_CONNECTION",
                },
                {
                    "trigger": "start_command",
                    "source": ["DRAWING", "ERASING"],
                    "dest": "ERROR",
                },
                {
                    "trigger": "connect",
                    "source": "DRAWING",
                    "dest": "ERROR",
                },
            ],
        )

        self.command = None
        self.paused = False

        self.last_heartbeat = None

        self.queue = []

        self.completed_drawings = []

        # Timeout for the watchdog thread
        self.watchdog_timeout_seconds_READY = 10
        self.watchdog_timeout_seconds_DRAWING = 300
        self.watchdog = threading.Thread(target=self._watchdog_thread)
        self.watchdog_active = False

        self.video_thread = None
        self.picture_thread = None
        self.picture_event = threading.Event()

        self.watchdog.start()

        self.drawing_success = False
        self.record_while_drawing = False

        self.camera = None

        self.cooldown_start = 0
        self.cooldown_time = 0

        # Register the state machine callbacks

    def set_camera(self, camera):
        self.camera = camera

        # Set the picture thread
        self.picture_thread = threading.Thread(target=self._picture_thread)
        self.picture_thread.start()

    def capture_picture(self):
        if self.camera is not None:
            # Generate the name
            name = self.next_gcode().get_name()

            if len(self.queue) > 0:
                output_dir = self.next_drawing().get_artifact_dir()

            if output_dir is not None:
                self.camera.get_frame(output_dir, name)

    def add_drawing(self, drawing):
        self.queue.append(drawing)

        # Start processing for the drawing
        drawing.start_processing()

    def update_ip(self, ip: str):
        self.ip = ip

    def __str__(self) -> str:
        return f"EtchBot({self.name})"

    def __repr__(self) -> str:
        return f"EtchBot({self.name})"

    def _update_heartbeat(self):
        self.last_heartbeat = time.time()

    def handle_connect_request(self):
        if self.may_connect():
            self.connect()
        else:
            print(
                f"Cannot connect to {self.name}. Robot is not in the correct state, error."
            )
            self.error()

        if self.state == "READY":
            return True

        return False

    def get_command(self):
        # Log the current time as the last heartbeat
        self._update_heartbeat()

        # Try to get a command based on the current state
        try:
            print("Getting command based on current state.")
            self.start_command()

            if self.state != "ERROR":
                return self.command

            else:
                # Print an error message if the robot is in an error state
                print(
                    f"{self.name} is in an error state as it fell out of sync with the state machine."
                )

                return None

        except transitions.MachineError:
            # If the condition is not met or the state machine is in an invalid state, return None
            return "invalid"

    def get_queue_information(self) -> list:
        """Returns information about the current queue.

        Each item in the queue will contain the following information:
        - Name of the drawing
        - Number of Gcodes in the drawing
        - Status of the drawing (Ready or not)

        Returns: list
        """

        queue_information = []

        for drawing in self.queue:
            ready, length = drawing.status()
            queue_information.append(
                {"name": drawing.name, "length": length, "ready": ready}
            )

        return queue_information

    def get_completed_information(self) -> list:
        """Returns information about the completed drawings.

        Each item in the list will contain the following information:
        - Name of the drawing
        - Number of Gcodes in the drawing
        - Status of the drawing (Ready or not)

        Returns: list
        """

        completed_information = []

        for drawing in self.completed_drawings:
            ready, length = drawing.status()
            completed_information.append({"name": drawing.name, "length": length})

        return completed_information

    def ready_to_draw(self, event):
        print(f"{self.name} is checking if a command is ready.")
        # Print length of the queue and the current state
        print(f"Queue length: {len(self.queue)}")
        print(f"Current state: {self.paused}")

        # Conditions:
        # - The queue is not empty
        # - The robot is not paused
        # - The screen is erased
        return (
            len(self.queue) > 0
            and len(self.queue[0]) > 0
            and not self.paused
            and self.is_screen_erased()
        )

    def gcode_ready(self) -> bool:
        # Return true if we are in drawing state AND the queue is not empty
        return (
            self.state == "DRAWING" and len(self.queue) > 0 and len(self.queue[0]) > 0
        )

    def next_gcode(self):
        # Return the GCode at the front of the queue
        if len(self.queue) > 0:
            return self.queue[0].next()

    def next_drawing(self):
        # Return the next drawing in the queue
        if len(self.queue) > 0:
            return self.queue[0]

    def ready_to_erase(self, event):
        return not self.paused and self.drawing_success

    def is_paused(self, event):
        return self.paused

    def pause(self):
        self.paused = True

    def resume(self):
        self.paused = False

    # State machine methods
    def on_enter_WAITING_FOR_CONNECTION(self, event):
        print(f"Ready to accept connection for {self.name}.")

        self.watchdog_active = False

    def on_enter_READY(self, event):
        print(f"{self.name} is ready to draw.")

        self._update_heartbeat()

        # Start the watchdog thread
        self.watchdog_active = True
        self.watchdog_timeout_seconds = self.watchdog_timeout_seconds_READY

    def on_enter_DRAWING(self, event):
        print(f"{self.name} is drawing.")

        # Reset the GCode at the beginning of the queue
        self.next_gcode().reset()

        # Start recording if enabled
        if self.record_while_drawing and self.camera is not None:
            self.camera.start_video_recording()

        # We need to send the GCode to the GCode server to queue it up
        # We are not expecting an immediate response during drawing phase
        self.watchdog_timeout_seconds = self.watchdog_timeout_seconds_DRAWING

        self.command = "draw"

    def on_enter_POST_DRAWING_ACTIONS(self, event):
        print(f"{self.name} has finished drawing.")

        # Set the drawing complete flag
        self.drawing_success = True

        # Stop the watchdog thread
        self.watchdog_active = False

        # Clear the command
        self.command = None

        # Start cooldown timer
        self.cooldown_start = time.time()
        drawing_time = event.kwargs.get("drawing_time", 30)

        self.cooldown_time = drawing_time * Config().get("robot.cooldown_multiplier", 1)

        # Check if the robot is recording
        if self.is_recording():
            output_dir = None
            name = None

            if len(self.queue) > 0:
                name = self.next_gcode().get_name()
                output_dir = self.next_drawing().get_artifact_dir()

            if output_dir is not None:
                self.camera.stop_video_recording(output_dir, name + "_video")
            else:
                print("Output directory is None. Video will be lost.")
                self.camera.stop_video_recording()

        # Check if there is a camera connected
        if self.picture_thread is not None:
            print("Setting picture event.")
            # Signal picture thread to capture a picture
            self.picture_event.set()

        else:
            # We're done with this Gcode, remove it and move to the next action
            self._remove_gcode()

            # Transition to the next state
            self.post_drawing_actions_complete()

    def get_completed_zip(self, completed_drawings_index: int):
        # Confirm that the index is valid
        if completed_drawings_index < 0 or completed_drawings_index >= len(
            self.completed_drawings
        ):
            return None

        return self.completed_drawings[completed_drawings_index].get_zipped_artifact()

    def _remove_gcode(self):
        # Remove the drawing from the queue
        cur_drawing = self.queue[0]
        cur_drawing.pop()

        # Do nothing and move to the next action
        # Check if the drawing is complete
        if len(cur_drawing) == 0:
            # Drawing is complete, remove it from the queue and move it to the completed drawings
            drawing = self.queue.pop(0)

            drawing.generate_video()
            self.completed_drawings.append(drawing)

    def on_enter_ERASING(self, event):
        print(f"{self.name} is erasing.")

        # Change the watchdog timeout
        # We don't expect a response from the robot during erasing
        self.watchdog_timeout_seconds = self.watchdog_timeout_seconds_DRAWING

        self.command = "erase"

    def on_enter_POST_ERASING_ACTIONS(self, event):
        print(f"{self.name} has finished erasing.")

        # Set the drawing complete flag, ready for the next
        self.drawing_success = False

        self.watchdog_active = False

        # Clear the command
        self.command = ""

        # Transition to the next state
        self.post_erasing_actions_complete()

    def is_screen_erased(self):
        return not self.drawing_success

        self.drawing_success = False

    def cooldown_complete(self, event):
        return self.cooldown_remaining() == 0

    def cooldown_remaining(self):
        time_delta = self.cooldown_time - (time.time() - self.cooldown_start)

        if time_delta < 0:
            return 0

        return time_delta

    def on_enter_ERROR(self, event):
        print(f"{self.name} has encountered an error.")
        # Stop the watchdog thread
        self.watchdog_active = False
        self.command = None

        if self.camera is not None and self.is_recording():
            # Stop recording, throw away the video
            self.camera.stop_video_recording()

    def on_exit_ERROR(self, event):
        print(f"{self.name} is recovering from an error.")

    def _watchdog_thread(self):
        while True:
            if self.watchdog_active and self.last_heartbeat is not None:
                if time.time() - self.last_heartbeat > self.watchdog_timeout_seconds:
                    self.timeout()

            time.sleep(1)

    def _picture_thread(self):
        while True:
            self.picture_event.wait()
            self.picture_event.clear()

            self.capture_picture()

            self._remove_gcode()

            # Signal that the picture has been captured, robot is ready to draw
            self.post_drawing_actions_complete()

    def enable_recording(self):
        self.record_while_drawing = True

    def disable_recording(self):
        self.record_while_drawing = False

    def is_recording(self):
        if self.camera is not None:
            return self.camera.is_recording

        return False


class EtchBotStore:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super(EtchBotStore, cls).__new__(cls)
            cls._instance._initialize(*args, **kwargs)
        return cls._instance

    def _initialize(self):
        # Store the robots in a dictionary by IP address AND name
        self.name_dict = {}
        self.ip_dict = {}

    def add_robot(self, robot: "EtchBot"):
        # Check if the robot is already in the store
        if robot.name in self.name_dict:
            raise ValueError(f"Robot with name {robot.name} already exists.")

        if robot.ip in self.ip_dict:
            # Remove the existing robot with the same IP address
            existing_robot = self.ip_dict[robot.ip]
            self.remove_robot(existing_robot)

            # Raise a warning if the existing robot has a different name
            print(
                f"Warning: Robot with IP address {robot.ip} already exists with name {existing_robot.name}."
            )
            print(f"Replacing with robot with name {robot.name}.")

        self.name_dict[robot.name] = robot
        self.ip_dict[robot.ip] = robot

    def get_robot_by_name(self, name) -> "EtchBot":
        return self.name_dict.get(name, None)

    def get_robot_by_ip(self, ip: str) -> "EtchBot":
        if ip is None or type(ip) is not str:
            return None
        return self.ip_dict.get(ip, None)

    def get_all_robots(self):
        return self.name_dict.values()

    def get_all_names(self) -> list:
        return list(self.name_dict.keys())

    def remove_robot(self, robot: "EtchBot"):
        if robot.name in self.name_dict:
            del self.name_dict[robot.name]

        if robot.ip in self.ip_dict:
            del self.ip_dict[robot.ip]

    def __str__(self) -> str:
        return f"EtchBotStore({self.name_dict})"

    def __repr__(self) -> str:
        return f"EtchBotStore({self.name_dict})"

    def __contains__(self, name: str):
        return name in self.name_dict
