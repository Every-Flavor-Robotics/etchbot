import requests
from pathlib import Path


class HTTPCamera:
    def __init__(self, url, camera_index):
        self.url = url
        self.camera_index = camera_index
        self.is_recording = False

        if not self._check_connection():
            raise Exception(f"Failed to connect to camera service at {url}")

    # Create a method to validate that a service is available at the given URL
    def _check_connection(self):
        try:
            # send GET to /available_webcams and confirm that the camera index is in the response
            response = requests.get(f"{self.url}/available_webcams")
            # Loop over the response and check if the camera index is in the response
            for camera in response.json():
                if camera["id"] == self.camera_index:
                    return True

            return False

        except requests.exceptions.ConnectionError:
            return False

    def get_frame(self, save_dir: Path, name="frame"):

        # Confirm that the save_dir exists and is a directory
        if not save_dir.is_dir():
            raise Exception(f"Invalid directory: {save_dir}")

        # Send a GET request to /get_frame/<int:cam_id> and return the response
        try:
            response = requests.get(f"{self.url}/capture_image/{self.camera_index}")
            # Get file extension from the response headers
            extension = response.headers["Content-Type"].split("/")[-1]
        except requests.exceptions.ConnectionError:
            print("Connection error")
            return None

        # Save the response content to a file in the save_dir
        save_path = save_dir / f"{name}.{extension}"
        with open(save_path, "wb") as file:
            file.write(response.content)

        return save_path

    def start_video_recording(self):
        # Send a GET request to /start_video_recording/<int:cam_id> and return the response
        try:
            response = requests.get(
                f"{self.url}/start_video_recording/{self.camera_index}"
            )
        except requests.exceptions.ConnectionError:
            return False

        self.is_recording = True

        # Return whether or not the response was successful
        return response.ok

    def stop_video_recording(self, save_dir: Path, name="video"):
        # Confirm that the save_dir exists and is a directory
        if not save_dir.is_dir():
            raise Exception(f"Invalid directory: {save_dir}")

        self.is_recording = False



        # Send a GET request to /stop_video_recording/<int:cam_id> and return the response
        try:
            response = requests.get(
                f"{self.url}/stop_video_recording/{self.camera_index}"
            )
        except requests.exceptions.ConnectionError:
            return None

        if response.status_code == 200:
            video_file_path = f"{name}.mp4"
            with open(video_file_path, "wb") as file:
                for chunk in response.iter_content(chunk_size=1024):
                    if chunk:
                        file.write(chunk)

        return save_path
