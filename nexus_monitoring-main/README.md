# Delivery GPS Localization

A local Python GUI for visualizing Arduino GPS + IMU data in real time.

## Overview

This project connects an Arduino-based GPS/IMU sensor to a desktop application and displays:
- live GPS coordinates on an OpenStreetMap tile map
- a trail of recorded GPS positions
- roll/pitch/yaw attitude from an IMU
- a rotating 3D robot model showing current orientation
- a compass and attitude indicator
- CSV recording support for logged data

## Main Components

- `gps_imu_tracker.py` — main Tkinter application. Handles serial communication, UI, map rendering, recording, and demo mode.
- `robot3dWidget.py` — custom visualization widgets:
  - `Robot3DWidget` for 3D attitude rendering
  - `AI` attitude indicator
  - `Compass` display
  - `MapWidget` for OpenStreetMap tile rendering and pan/zoom/trail support
- `additional_func.py` — GPS coordinate conversion and tile fetching utilities.
- `config.py` — UI theme colors and OpenStreetMap tile configuration.
- `delivery_GPS_localization/delivery_GPS_localization.ino` — Arduino sketch that reads GPS and MPU6050 IMU data, computes roll/pitch/yaw, and sends serial output.
- `requirements.txt` — Python dependencies.

## Requirements

Install the required Python packages:

```bash
pip install -r requirements.txt
```

Required packages:
- `numpy`
- `matplotlib`
- `Pillow`
- `pyserial`

The application also uses the standard library `tkinter` for the GUI.

## Usage

1. Upload `delivery_GPS_localization/delivery_GPS_localization.ino` to an Arduino with a GPS module and MPU6050.
2. Connect the Arduino to your PC.
3. Run the desktop app:

```bash
python gps_imu_tracker.py
```

4. In the app:
- select the serial port and baud rate
- click `CONNECT`
- view live GPS/IMU data
- use `RECORD` to save CSV log data
- use `DEMO` to run a simulated path without hardware

## Notes

- The map uses OpenStreetMap tiles and fetches them dynamically.
- If `Pillow` or `pyserial` are not installed, the app prints a helpful install command.
- Demo mode is useful for testing UI behavior without a live Arduino connection.

## Folder Structure

- `gps_imu_tracker.py` — desktop tracker app
- `robot3dWidget.py` — 3D and map visualization widgets
- `additional_func.py` — map helper functions
- `config.py` — styling and tile settings
- `requirements.txt` — Python dependency list
- `delivery_GPS_localization/` — Arduino sketch source

## License

No license is specified. Use and modify the code according to your own needs.
