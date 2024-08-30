import asyncio
from pathlib import Path
from open_gopro import Params, WiredGoPro, WirelessGoPro, proto
from open_gopro.gopro_base import GoProBase
from open_gopro.logger import setup_logging
from rich.console import Console

console = Console()


class GoProWrapper:
    def __init__(
        self,
        identifier: str,
        wired: bool = False,
        wifi_interface: str = None,
        log_level: str = "INFO",
    ):
        self.identifier = identifier
        self.wired = wired
        self.wifi_interface = wifi_interface
        self.logger = setup_logging(__name__, log_level)
        self.gopro: GoProBase | None = None

    async def connect(self):
        try:
            if self.wired:
                self.gopro = WiredGoPro(self.identifier)  # type: ignore
            else:
                self.gopro = WirelessGoPro(
                    self.identifier, wifi_interface=self.wifi_interface
                )

            # Connect to the GoPro
            await self.gopro.connect()

            if not self.gopro:
                raise Exception("Failed to connect to GoPro.")

            console.print(f"Connected to GoPro: {self.identifier}")

        except Exception as e:
            self.logger.error(f"Error connecting to GoPro: {repr(e)}")
            if self.gopro:
                await self.gopro.close()

    async def disconnect(self):
        if self.gopro:
            await self.gopro.close()
            console.print(f"Disconnected from GoPro: {self.identifier}")

    async def take_photo(self, output: Path = Path("photo.jpg")):
        if not self.gopro:
            raise Exception("GoPro is not connected.")

        try:
            assert (
                await self.gopro.http_command.load_preset_group(
                    group=proto.EnumPresetGroup.PRESET_GROUP_ID_PHOTO
                )
            ).ok

            media_set_before = set(
                (await self.gopro.http_command.get_media_list()).data.files
            )

            assert (
                await self.gopro.http_command.set_shutter(shutter=Params.Toggle.ENABLE)
            ).ok

            media_set_after = set(
                (await self.gopro.http_command.get_media_list()).data.files
            )
            photo = media_set_after.difference(media_set_before).pop()

            console.print(f"Downloading {photo.filename}...")
            await self.gopro.http_command.download_file(
                camera_file=photo.filename, local_file=output
            )
            console.print(
                f"Success!! :smiley: File has been downloaded to {output.absolute()}"
            )

        except Exception as e:
            self.logger.error(f"Error taking photo: {repr(e)}")

    async def start_video(self):
        if not self.gopro:
            raise Exception("GoPro is not connected.")

        try:
            assert (
                await self.gopro.http_command.load_preset_group(
                    group=proto.EnumPresetGroup.PRESET_GROUP_ID_VIDEO
                )
            ).ok
            assert (
                await self.gopro.http_command.set_shutter(shutter=Params.Toggle.ENABLE)
            ).ok
            console.print(f"Video recording started on {self.identifier}")

        except Exception as e:
            self.logger.error(f"Error starting video recording: {repr(e)}")

    async def stop_video(self):
        if not self.gopro:
            raise Exception("GoPro is not connected.")

        try:
            assert (
                await self.gopro.http_command.set_shutter(shutter=Params.Toggle.DISABLE)
            ).ok
            console.print(f"Video recording stopped on {self.identifier}")

        except Exception as e:
            self.logger.error(f"Error stopping video recording: {repr(e)}")

    async def download_last_video(self, output: Path = Path("video.mp4")):
        if not self.gopro:
            raise Exception("GoPro is not connected.")

        try:
            media_list = (await self.gopro.http_command.get_media_list()).data.files
            last_video = sorted(
                media_list, key=lambda x: x["createtime"], reverse=True
            )[0]

            console.print(f"Downloading {last_video.filename}...")
            await self.gopro.http_command.download_file(
                camera_file=last_video.filename, local_file=output
            )
            console.print(
                f"Success!! :smiley: File has been downloaded to {output.absolute()}"
            )

        except Exception as e:
            self.logger.error(f"Error downloading video: {repr(e)}")
