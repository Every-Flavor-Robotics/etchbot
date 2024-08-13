import transitions


class EtchBot:
    def __init__(self, name):
        # Name of the robot
        self.name = name

        # Robot states:
        # - DISCONNECTED: The robot is not connected
        # - READY: The robot is connected and ready to draw
        # - DRAWING: The robot is drawing
        # - DRAWING_COMPLETE: The robot has finished drawing, ready to erase
        # - ERASING: The robot is erasing
        # - ERASING_COMPLETE: The robot has finished erasing
        # - REBOOTING: The robot is rebooting
        # - ERROR: The robot has encountered an error that requires manual intervention

        self.machine = transitions.Machine(
            model=self,
            states=[
                "DISCONNECTED",
                "READY",
                "DRAWING",
                "DRAWING_COMPLETE",
                "ERASING",
                "ERASING_COMPLETE",
                "ERROR",
            ],
            initial="DISCONNECTED",
            transitions=[
                {"trigger": "connect", "source": "DISCONNECTED", "dest": "READY"},
                {"trigger": "draw", "source": "READY", "dest": "DRAWING"},
                {"trigger": "stop", "source": "DRAWING", "dest": "DRAWING_COMPLETE"},
                {"trigger": "erase", "source": "DRAWING_COMPLETE", "dest": "ERASING"},
                {"trigger": "stop", "source": "ERASING", "dest": "ERASING_COMPLETE"},
                {"trigger": "reboot", "source": "*", "dest": "DISCONNECTED"},
                {"trigger": "error", "source": "*", "dest": "ERROR"},
                {"trigger": "recover", "source": "ERROR", "dest": "READY"},
            ],
        )

        # Register the state machine callbacks

    def __str__(self) -> str:
        return f"EtchBot({self.name})"

    def __repr__(self) -> str:
        return f"EtchBot({self.name})"

    # State machine methods
    def on_enter_DISCONNECTED(self):
        print(f"{self.name} is disconnected.")

    def on_enter_READY(self):
        print(f"{self.name} is ready to draw.")

    def on_enter_DRAWING(self):
        print(f"{self.name} is drawing.")

    def on_enter_DRAWING_COMPLETE(self):
        print(f"{self.name} has finished drawing.")

    def on_enter_ERASING(self):
        print(f"{self.name} is erasing.")

    def on_enter_ERASING_COMPLETE(self):
        print(f"{self.name} has finished erasing.")

    def on_enter_ERROR(self):
        print(f"{self.name} has encountered an error.")

    def on_exit_ERROR(self):
        print(f"{self.name} is recovering from an error.")
