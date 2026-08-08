import re
import socket


def get_station_number():
    hostname = socket.gethostname()
    match = re.search(r'robotics-education-lab(\d+)(?:\..*)?$', hostname)
    if match:
        return int(match.group(1))
    return None


def get_station_namespace():
    station_number = get_station_number()
    if station_number is not None:
        return f'station{station_number}'
    return 'station'


def default_rs_color_topic(camera_node_name: str) -> str:
    """Absolute color topic under PushRosNamespace(station), e.g. /station11/.../color/image_raw."""
    ns = get_station_namespace()
    return f"/{ns}/{camera_node_name}/color/image_raw"

