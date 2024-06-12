import pygame
import cv2
import numpy as np
from typing import Optional
from pathlib import Path
import time
import re
import threading
from filelock import FileLock, Timeout

UPLOAD_DIR = Path("uploads")
PROCESSING_DIR = Path("processing")
OUTPUT_DIR = Path("gcode_outputs")


status_UNKNOWN = "UNKNOWN"
status_READY = "READY"
status_DRAWING = "DRAWING"
status_ERROR = "ERROR"
robot_status = status_UNKNOWN


def get_robot_status():
    global robot_status
    # Open filelock
    with FileLock("robot_status.txt.lock"):
        with open("robot_status.txt") as f:
            cur_status = f.read()
    return cur_status


# Initialize Pygame
pygame.init()

# Font for the status bar text
font = pygame.font.Font(None, 36)
background_color = [145, 145, 139]
primary = [72, 104, 35]
secondary = [139, 148, 125]
tertiary = [114, 152, 149]
error = [255, 84, 73]

# Get information about the display
info_object = pygame.display.Info()
screen_width, screen_height = 1920, 1080

# Set up the display for fullscreen
# window = pygame.display.set_mode((screen_width, screen_height), pygame.FULLSCREEN)
window = pygame.display.set_mode((screen_width, screen_height))
pygame.display.set_caption("Robot Control GUI")

# Open etch.png
# Define states
READY = "READY"
PROCESSING = "PROCESSING"
DRAWING = "DRAWING"
ERROR = "Error"
state_machine_status = READY

# Define state transitions
SM_SUCCESS = "SUCCESS"
SM_FAILURE = "FAILURE"
SM_NONE = "NONE"


# Create two threads to read from the cameras
import threading


class CamThread(threading.Thread):
    def __init__(self, previewName, camID):
        threading.Thread.__init__(self)
        self.previewName = previewName
        self.camID = camID
        self.lock = threading.Lock()
        self.cur_frame = None

    def run(self):

        # Initialize the camera
        cap = cv2.VideoCapture(self.camID)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        while True:
            ret, frame = cap.read()
            if ret:
                with self.lock:
                    self.cur_frame = frame

    def get_frame(self):
        with self.lock:
            return self.cur_frame


def last_update_time(file_path: Path) -> float:
    """Get the last time the file was modified."""
    return file_path.stat().st_mtime


# Initialize the camera
center_x = screen_width // 2
center_y = screen_height // 2
camera_width = 1280
camera_height = 720

cap_picture_thread = CamThread("Picture", "/dev/video0")
cap_picture_thread.start()

cap_etch_thread = CamThread("Etch", "/dev/video4")
cap_etch_thread.start()

# Create a VideoWriter
fourcc = cv2.VideoWriter_fourcc(*"XVID")


def resize_with_aspect_ratio(image, height):
    """Resize an image with a given height, maintaining the aspect ratio."""
    aspect_ratio = image.shape[1] / image.shape[0]
    width = int(height * aspect_ratio)
    return cv2.resize(image, (width, height))


def clear_screen(window):
    window.fill((0, 0, 0))


def draw_ready_screen(
    window, status_text, events, out
) -> tuple[str, Optional[np.ndarray]]:
    """Draws the screen content for the STANDBY state."""
    global cap_picture_thread, center_x, center_y, camera_width, camera_height

    # Read frame from the camera
    frame = cap_picture_thread.get_frame()
    if frame is None:
        return SM_NONE, None
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame_resized = np.rot90(frame)
    frame_resized = pygame.surfarray.make_surface(frame_resized)
    window.blit(
        frame_resized, (center_x - camera_width // 2, center_y - camera_height // 2)
    )

    # Check if space bar is release
    for event in events:
        if event.type == pygame.KEYUP and event.key == pygame.K_SPACE:
            return SM_SUCCESS, frame

    return SM_NONE, None


frame_local = None
cur_frame_name = None
last_preprocessed_update = None


def draw_processing_screen(window, status_text, events, out):
    """Draws the screen content for the PROCESSING state."""
    global frame_local, cur_frame_name, last_preprocessed_update, robot_status

    if out is not None:
        frame_local = cv2.cvtColor(out, cv2.COLOR_RGB2BGR)
        # Write the frame to the UPLOAD_DIR - include time stamp in the file name
        cur_frame_name = f"gui_frame_{int(time.time())}"
        cv2.imwrite(
            str(UPLOAD_DIR / (cur_frame_name + ".png")),
            cv2.cvtColor(out, cv2.COLOR_RGB2BGR),
        )

    # Check if preprocessed.png exists and is newer than the last update
    # If so, open and save to frame_local
    # Remove png from cur_frame_name

    preprocess_gcode = PROCESSING_DIR / cur_frame_name / "gcode.png"
    preprocess_file = PROCESSING_DIR / cur_frame_name / "preprocessed.png"
    final_gcode = PROCESSING_DIR / cur_frame_name / "final.gcode"
    if preprocess_gcode.exists():
        if (
            last_preprocessed_update is None
            or last_update_time(preprocess_gcode) > last_preprocessed_update
        ):
            try:
                with FileLock(str(preprocess_gcode) + ".lock"):
                    frame_local = cv2.imread(str(preprocess_gcode))

                last_preprocessed_update = (preprocess_gcode).stat().st_mtime
            except Timeout:
                pass

    elif preprocess_file.exists() or preprocess_gcode.exists():

        if (
            last_preprocessed_update is None
            or last_update_time(preprocess_file) > last_preprocessed_update
        ):
            try:
                with FileLock(str(preprocess_gcode) + ".lock"):
                    frame_local = cv2.imread(str(preprocess_file))
                last_preprocessed_update = (preprocess_file).stat().st_mtime
            except Timeout:
                pass

    if robot_status == status_DRAWING:
        # Reset state variables
        out = cur_frame_name
        frame_local = None
        cur_frame_name = None
        last_preprocessed_update = None
        return SM_SUCCESS, out

    # Display frame_local centered on the screen
    if frame_local is not None:
        # Draw black background behind the image
        pygame.draw.rect(
            window,
            (0, 0, 0),
            (
                center_x - camera_width // 2,
                center_y - camera_height // 2,
                camera_width,
                camera_height,
            ),
        )
        frame_resized = resize_with_aspect_ratio(frame_local, camera_height)
        frame_resized = np.rot90(frame_resized)
        frame_resized = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)
        # Calculate how much to offset the image to center it
        delta = (camera_width - frame_resized.shape[0]) // 2
        frame_resized = pygame.surfarray.make_surface(frame_resized)

        window.blit(
            frame_resized,
            (center_x - camera_width // 2 + delta, center_y - camera_height // 2),
        )

    return SM_NONE, None


video_out = None
start_time = None


def draw_drawing_screen(window, status_text, events, out):

    global video_out, start_time, fourcc, cur_frame_name, robot_status

    if video_out is None:
        cur_frame_name = out
        start_time = time.time()
        video_out = cv2.VideoWriter(
            str(PROCESSING_DIR / f"{cur_frame_name}" / "etch_video.avi"),
            fourcc,
            30.0,
            (1280, 720),
        )

    # Read frame from the etch camera
    frame = cap_etch_thread.get_frame()
    if frame is not None:
        # Write frame to video_out
        video_out.write(frame)

        # Display frame on the screen
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = np.rot90(frame)
        frame = pygame.surfarray.make_surface(frame)

        window.blit(
            frame, (center_x - camera_width // 2, center_y - camera_height // 2)
        )
    else:
        print("No frame found.")

    if robot_status == status_READY:
        video_out.release()
        video_out = None
        start_time = None
        return SM_SUCCESS, "Done"

    elif robot_status == status_ERROR:
        video_out.release()
        video_out = None
        start_time = None
        return SM_FAILURE, "Error"

    return SM_NONE, None


def draw_error_screen(window, status_text):
    """Draws the screen content for the ERROR state."""
    window.fill((0, 0, 0))  # Red background
    # Write error in error color in the center of the screeen
    error_text = font.render("Error, please reset robot and restart app", True, error)
    error_text_rect = error_text.get_rect()
    error_text_rect.center = (screen_width // 2, screen_height // 2)
    window.blit(error_text, error_text_rect)
    window.blit(status_text, status_text_rect)

    return SM_NONE, None


# Dictionary mapping states to their drawing functions
state_to_draw_func = {
    READY: draw_ready_screen,
    PROCESSING: draw_processing_screen,
    DRAWING: draw_drawing_screen,
    ERROR: draw_error_screen,
}

STATE_FLOW = {
    READY: {
        SM_SUCCESS: PROCESSING,
        SM_FAILURE: ERROR,
    },
    PROCESSING: {
        SM_SUCCESS: DRAWING,
        SM_FAILURE: ERROR,
    },
    DRAWING: {
        SM_SUCCESS: READY,
        SM_FAILURE: ERROR,
    },
    ERROR: {
        SM_SUCCESS: READY,
        SM_FAILURE: ERROR,
    },
}


running = True
clock = pygame.time.Clock()
target_fps = 30  # Set the desired FPS

out = None
while running:
    window.fill(background_color)
    events = pygame.event.get()

    robot_status = get_robot_status()

    # Render robot status text
    status_text = font.render(f"Robot Status: {robot_status}", True, (255, 255, 255))
    status_text_rect = status_text.get_rect()
    status_text_rect.topleft = (10, 10)  # Position the text (adjust as needed)

    # Draw the screen based on the current state
    status, out = state_to_draw_func[state_machine_status](
        window, status_text, events, out
    )

    if status == SM_SUCCESS:
        state_machine_status = STATE_FLOW[state_machine_status][SM_SUCCESS]

        clear_screen(window)

    # Draw status bar background
    bar_rect = pygame.Rect(0, 0, screen_width, status_text_rect.height + 20)
    pygame.draw.rect(window, primary, bar_rect)  # Black background

    # In the center of the screen, draw a bounding box for the camera feed
    # 1280 x 720

    # Draw the bounding box with secondary color
    pygame.draw.rect(
        window,
        primary,
        (
            center_x - camera_width // 2,
            center_y - camera_height // 2,
            camera_width,
            camera_height,
        ),
        20,
    )

    # Blit the status text onto the window
    window.blit(status_text, status_text_rect)

    pygame.display.update()
    clock.tick(target_fps)

cap_picture.release()
pygame.quit()
