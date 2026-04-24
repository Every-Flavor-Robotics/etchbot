#!/usr/bin/env python3
"""
WebSocket-to-Serial Bridge for EtchBot.

Runs on the Windows host machine. Bridges between:
  - Docker container (WebSocket client connects to ws://host.docker.internal:9876)
  - ESP32 microcontroller (USB Serial, e.g., COM3)

Usage:
  python serial_bridge.py --port COM3
  python serial_bridge.py --port COM3 --baud 115200 --ws-port 9876
"""

import argparse
import asyncio
import sys
import signal

import serial
import serial.tools.list_ports
import websockets


class SerialBridge:
    def __init__(self, serial_port: str, baud_rate: int = 115200):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.ser = None
        self.ws_client = None

    def open_serial(self):
        """Open the serial connection to the ESP32."""
        self.ser = serial.Serial(
            self.serial_port,
            self.baud_rate,
            timeout=0.1,
        )
        print(f"[bridge] Serial port {self.serial_port} opened at {self.baud_rate} baud")

    async def handle_ws_client(self, websocket):
        """Handle a single WebSocket client (the Docker server)."""
        print(f"[bridge] Docker server connected via WebSocket")
        self.ws_client = websocket

        # Launch two tasks: serial→ws and ws→serial
        serial_to_ws_task = asyncio.create_task(self._serial_to_ws(websocket))
        ws_to_serial_task = asyncio.create_task(self._ws_to_serial(websocket))

        try:
            # Wait for either task to complete (usually means disconnect)
            done, pending = await asyncio.wait(
                [serial_to_ws_task, ws_to_serial_task],
                return_when=asyncio.FIRST_COMPLETED,
            )
            for task in pending:
                task.cancel()
        except Exception as e:
            print(f"[bridge] Session error: {e}")
        finally:
            self.ws_client = None
            print(f"[bridge] Docker server disconnected")

    async def _serial_to_ws(self, websocket):
        """Read lines from serial and forward to WebSocket."""
        loop = asyncio.get_event_loop()
        buffer = ""

        while True:
            try:
                # Read available bytes from serial in a thread (non-blocking)
                data = await loop.run_in_executor(
                    None, lambda: self.ser.read(self.ser.in_waiting or 1)
                )
                if data:
                    buffer += data.decode("utf-8", errors="replace")

                    # Process complete lines
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        line = line.strip()
                        if line:
                            print(f"[ESP32 → Server] {line}")
                            await websocket.send(line + "\n")
                else:
                    await asyncio.sleep(0.01)
            except serial.SerialException as e:
                print(f"[bridge] Serial read error: {e}")
                break
            except websockets.exceptions.ConnectionClosed:
                print(f"[bridge] WebSocket closed during serial read")
                break

    async def _ws_to_serial(self, websocket):
        """Read messages from WebSocket and forward to serial."""
        try:
            async for message in websocket:
                line = message.strip()
                if line:
                    print(f"[Server → ESP32] {line}")
                    self.ser.write((line + "\n").encode("utf-8"))
                    self.ser.flush()
        except websockets.exceptions.ConnectionClosed:
            print(f"[bridge] WebSocket closed during WS read")

    async def run(self, ws_host: str = "0.0.0.0", ws_port: int = 9876):
        """Start the WebSocket server and wait for connections."""
        self.open_serial()

        print(f"[bridge] WebSocket server listening on ws://{ws_host}:{ws_port}")
        print(f"[bridge] Docker containers should connect to ws://host.docker.internal:{ws_port}")
        print(f"[bridge] Waiting for Docker server to connect...")

        async with websockets.serve(self.handle_ws_client, ws_host, ws_port):
            await asyncio.Future()  # Run forever


def list_serial_ports():
    """List available serial ports."""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found.")
    else:
        print("Available serial ports:")
        for port in ports:
            print(f"  {port.device}: {port.description}")


def main():
    parser = argparse.ArgumentParser(description="EtchBot WebSocket-to-Serial Bridge")
    parser.add_argument(
        "--port",
        type=str,
        help="Serial port (e.g., COM3 on Windows, /dev/ttyACM0 on Linux)",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Serial baud rate (default: 115200)",
    )
    parser.add_argument(
        "--ws-port",
        type=int,
        default=9876,
        help="WebSocket server port (default: 9876)",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List available serial ports and exit",
    )

    args = parser.parse_args()

    if args.list:
        list_serial_ports()
        sys.exit(0)

    if not args.port:
        print("Error: --port is required. Use --list to see available ports.")
        list_serial_ports()
        sys.exit(1)

    bridge = SerialBridge(args.port, args.baud)

    # Handle Ctrl+C gracefully
    def signal_handler(sig, frame):
        print("\n[bridge] Shutting down...")
        if bridge.ser and bridge.ser.is_open:
            bridge.ser.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    print(f"[bridge] Starting EtchBot Serial Bridge")
    print(f"[bridge] Serial: {args.port} @ {args.baud} baud")
    print(f"[bridge] WebSocket: 0.0.0.0:{args.ws_port}")
    print()

    asyncio.run(bridge.run(ws_port=args.ws_port))


if __name__ == "__main__":
    main()
