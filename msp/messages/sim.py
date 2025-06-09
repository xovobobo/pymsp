import ctypes
from typing import Any
from .base_msg import MSP_MESSAGE


class SimulatorFlags:
    HITL_RESET_FLAGS = 0 << 0
    HITL_ENABLE = 1 << 0
    HITL_SIMULATE_BATTERY = 1 << 1
    HITL_MUTE_BEEPER = 1 << 2
    HITL_USE_IMU = 1 << 3
    HITL_HAS_NEW_GPS_DATA = 1 << 4
    HITL_EXT_BATTERY_VOLTAGE = 1 << 5
    HITL_AIRSPEED = 1 << 6
    HITL_EXTENDED_FLAGS = 1 << 7
    HITL_GPS_TIMEOUT = 1 << 8
    HITL_PITOT_FAILURE = 1 << 9


class MSP_SIM_REQUEST(MSP_MESSAGE):
    ID = 0x201F
    _pack_ = 1
    _fields_ = [
        ("version", ctypes.c_uint8),
        ("flags", ctypes.c_uint8),
        ("fix", ctypes.c_uint8),
        ("numSat", ctypes.c_uint8),
        ("lat", ctypes.c_int32),
        ("lon", ctypes.c_int32),
        ("alt", ctypes.c_int32),
        ("speed", ctypes.c_int16),
        ("course", ctypes.c_int16),
        ("velNED", ctypes.c_int16 * 3),
        ("roll", ctypes.c_int16),
        ("pitch", ctypes.c_int16),
        ("yaw", ctypes.c_int16),
        ("accel_x", ctypes.c_int16),
        ("accel_y", ctypes.c_int16),
        ("accel_z", ctypes.c_int16),
        ("gyro_x", ctypes.c_int16),
        ("gyro_y", ctypes.c_int16),
        ("gyro_z", ctypes.c_int16),
        ("baro", ctypes.c_int32),
        ("mag_x", ctypes.c_int16),
        ("mag_y", ctypes.c_int16),
        ("mag_z", ctypes.c_int16),
        ("vbat", ctypes.c_uint8),
        ("airspeed", ctypes.c_uint16),
        ("flags2", ctypes.c_uint8),
    ]

    def __str__(self):
        return f"""MSP_SIM_REQUEST:
        Version: {self.version}
        Flags: {self.flags}
        Fix: {self.fix}
        Num Satellites: {self.numSat}
        Latitude: {self.lat}
        Longitude: {self.lon}
        Altitude: {self.alt}
        Speed: {self.speed}
        Course: {self.course}
        Velocity NED: [{self.velNED[0]}, {self.velNED[1]}, {self.velNED[2]}]
        Roll: {self.roll}
        Pitch: {self.pitch}
        Yaw: {self.yaw}
        Acceleration: [X: {self.accel_x}, Y: {self.accel_y}, Z: {self.accel_z}]
        Gyro: [X: {self.gyro_x}, Y: {self.gyro_y}, Z: {self.gyro_z}]
        Baro: {self.baro}
        Magnetometer: [X: {self.mag_x}, Y: {self.mag_y}, Z: {self.mag_z}]
        VBat: {self.vbat}
        Airspeed: {self.airspeed}
        Flags2: {self.flags2}"""

    def __init__(self, *args: Any, **kw: Any) -> None:
        super().__init__(*args, **kw)
        self.version = 2


class MSP_SIM_RESPONSE(MSP_MESSAGE):
    ID = 0x201F
    _pack_ = 1
    _fields_ = [
        ("roll", ctypes.c_int16),
        ("pitch", ctypes.c_int16),
        ("yaw", ctypes.c_int16),
        ("throttle", ctypes.c_int16),
        ("debug_index", ctypes.c_uint8),
        ("debug_value", ctypes.c_int32),
        ("estimated_attitude_roll", ctypes.c_int16),
        ("estimated_attitude_pitch", ctypes.c_int16),
        ("estimated_attitude_yaw", ctypes.c_int16),
    ]
