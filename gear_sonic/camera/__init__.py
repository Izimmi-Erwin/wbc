"""Camera server package for streaming camera images over ZMQ.

Runs on the robot computer and publishes JPEG-encoded frames that the
data exporter (on the workstation) subscribes to for recording.

Quickstart (on robot)::

    bash install_scripts/install_camera_server.sh
    source .venv_camera/bin/activate
    python -m gear_sonic.camera.composed_camera --ego-view-camera oak

See ``gear_sonic/scripts/run_data_exporter.py`` for the data-collection entry point.
"""

from gear_sonic.camera.sensor_server import (
    CameraMountPosition,
    ImageMessageSchema,
    ImageUtils,
    SensorClient,
    SensorServer,
)

__all__ = [
    "CameraMountPosition",
    "ImageMessageSchema",
    "ImageUtils",
    "SensorClient",
    "SensorServer",
]
