"""Room label parsing shared by the node and the tests."""
import math


class RoomError(ValueError):
    pass


def load_rooms(text):
    """Parse a rooms YAML document into [{name, x, y, yaw}]."""
    import yaml

    data = yaml.safe_load(text) or {}
    rooms = data.get('rooms') if isinstance(data, dict) else None
    if not isinstance(rooms, list):
        raise RoomError('rooms file must contain a "rooms" list')
    parsed = []
    seen = set()
    for index, item in enumerate(rooms):
        if not isinstance(item, dict):
            raise RoomError(f'room #{index} must be a mapping')
        name = str(item.get('name', '')).strip()
        if not name:
            raise RoomError(f'room #{index} has no name')
        if name in seen:
            raise RoomError(f'duplicate room name: {name}')
        seen.add(name)
        try:
            x = float(item['x'])
            y = float(item['y'])
        except (KeyError, TypeError, ValueError) as exc:
            raise RoomError(f'room {name} needs numeric x and y') from exc
        yaw = float(item.get('yaw', 0.0))
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise RoomError(f'room {name} has non-finite coordinates')
        parsed.append({'name': name, 'x': x, 'y': y, 'yaw': yaw})
    return parsed


def find_room(rooms, name):
    for room in rooms:
        if room['name'] == name:
            return room
    return None
