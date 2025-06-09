import ctypes

from .base_msg import MSP_MESSAGE
from typing import Any

class MSP_MOTOR_REQUEST(MSP_MESSAGE):
    ID = 104


class MSP_MOTOR_RESPONSE(MSP_MESSAGE):
    ID = 104
    _pack_ = 1
    _fields_ = [
        ('motors', ctypes.c_uint16 * 8),
    ]

    def __init__(self, *args: Any, **kw: Any) -> None:
        super().__init__(*args, **kw)