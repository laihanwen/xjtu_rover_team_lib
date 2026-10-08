"""2026105 joystick mapping; wire order follows STM32 RC.c, not old text table."""
import math
import json


def axis_byte(value):
    if not math.isfinite(value):
        raise ValueError("non-finite joystick axis")
    value = max(-1.0, min(1.0, value))
    return round(127 + value * (128 if value >= 0 else 127))


def switch_byte(value):
    return 0 if value < -0.5 else 2 if value > 0.5 else 1


def encode_frame(axes, buttons, dial_min=-0.5, dial_max=-0.1):
    if len(axes) < 7 or len(buttons) < 3:
        raise ValueError("controller requires 7 axes and 3 buttons")
    if not all(math.isfinite(x) for x in axes[:7]):
        raise ValueError("non-finite joystick input")
    if not math.isfinite(dial_min) or not math.isfinite(dial_max) or dial_max <= dial_min:
        raise ValueError("invalid dial calibration")
    dial = round(255 * max(0.0, min(1.0, (axes[4] - dial_min) / (dial_max - dial_min))))
    return bytes([0xA5, *(axis_byte(x) for x in axes[:4]), dial,
                  switch_byte(axes[6]), switch_byte(axes[5]),
                  int(bool(buttons[0])), int(bool(buttons[1])), int(bool(buttons[2]))])


def joystick_attached(pg, device):
    """pygame Joystick has no get_attached(); match the hotplug instance ID."""
    try:
        return device.get_init() and any(
            pg.joystick.Joystick(i).get_instance_id() == device.get_instance_id()
            for i in range(pg.joystick.get_count()))
    except pg.error:
        return False


def load_mapping(path):
    with open(path, encoding="utf-8") as source:
        mapping = json.load(source)
    axes = mapping.get("motion_axes")
    signs = mapping.get("motion_signs")
    if not isinstance(axes, list) or len(axes) != 4 or not all(type(x) is int and 0 <= x < 64 for x in axes):
        raise ValueError("mapping requires four valid motion_axes")
    if not isinstance(signs, list) or len(signs) != 4 or any(x not in (-1, 1) for x in signs):
        raise ValueError("mapping requires four motion_signs of +/-1")
    for key in ("speed_axis", "servo_select_axis", "yaw_button", "depth_button"):
        if type(mapping.get(key)) is not int or not 0 <= mapping[key] < 64:
            raise ValueError("invalid mapping index: " + key)
    dial = mapping.get("dial_axis")
    if dial is not None and (type(dial) is not int or not 0 <= dial < 64):
        raise ValueError("invalid dial_axis")
    if not isinstance(mapping.get("guid"), str) or not mapping["guid"]:
        raise ValueError("mapping requires device guid")
    deadman = mapping.get("deadman_axis", mapping["servo_select_axis"])
    if type(deadman) is not int or not 0 <= deadman < 64:
        raise ValueError("invalid deadman_axis")
    if dial is not None and deadman == mapping["servo_select_axis"]:
        raise ValueError("camera SC selector must be independent of deadman_axis")
    if dial is not None:
        low, high = mapping.get('dial_min'), mapping.get('dial_max')
        if (type(low) not in (int,float) or type(high) not in (int,float) or
                not math.isfinite(low) or not math.isfinite(high) or high <= low):
            raise ValueError('verified dial requires measured dial_min/dial_max')
    return mapping


def encode_mapped_frame(axes, buttons, mapping, dial_min=None, dial_max=None):
    if mapping is None:
        return encode_frame(axes, buttons, -.5 if dial_min is None else dial_min,
                            -.1 if dial_max is None else dial_max)
    dial_min = mapping.get('dial_min', -.5) if dial_min is None else dial_min
    dial_max = mapping.get('dial_max', -.1) if dial_max is None else dial_max
    try:
        canonical = [axes[i] * sign for i, sign in zip(mapping["motion_axes"], mapping["motion_signs"])]
        canonical += [axes[mapping["dial_axis"]] if mapping["dial_axis"] is not None else dial_min,
                      axes[mapping["servo_select_axis"]], axes[mapping["speed_axis"]]]
        switches = [buttons[mapping["yaw_button"]], buttons[mapping["depth_button"]], 0]
    except IndexError as error:
        raise ValueError("controller does not provide configured inputs") from error
    frame = encode_frame(canonical, switches, dial_min, dial_max)
    # Byte10 is a verified-camera-dial marker, never the removed main servo.
    if mapping["dial_axis"] is None:
        return frame[:5] + bytes([127]) + frame[6:10] + bytes([0])
    return frame[:10] + bytes([1])


class RemoteControl:
    def __init__(self, dial_min, dial_max, mapping=None):
        import pygame
        self.pg = pygame
        pygame.init()
        pygame.joystick.init()
        self.device = None
        self.dial_min, self.dial_max = dial_min, dial_max
        self.mapping = mapping

    def read(self):
        self.pg.event.pump()
        if self.device is not None and not joystick_attached(self.pg, self.device):
            self.device = None
        if self.device is None:
            if self.pg.joystick.get_count() == 0:
                return None
            self.device = self.pg.joystick.Joystick(0)
            if not self.device.get_init():
                self.device.init()
        if self.mapping and self.device.get_guid() != self.mapping["guid"]:
            raise ValueError("controller GUID does not match mapping configuration")
        axes = [self.device.get_axis(i) for i in range(self.device.get_numaxes())]
        buttons = [self.device.get_button(i) for i in range(self.device.get_numbuttons())]
        return encode_mapped_frame(axes, buttons, self.mapping, self.dial_min, self.dial_max)
