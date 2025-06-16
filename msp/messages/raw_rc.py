import ctypes
from typing import Any
from .base_msg import MSP_MESSAGE

class MSP_SET_RAW_RC_REQUEST(MSP_MESSAGE):
    ID = 200
    _pack_ = 1
    _no_reply = True
    _fields_ = [
        ('channels', ctypes.c_uint16 * 8),
    ]

    def __init__(self, *args: Any, **kw: Any) -> None:
        super().__init__(*args, **kw)

    def __str__(self):
        channel_values = ', '.join(str(channel) for channel in self.channels)
        return f"MSP_SET_RAW_RC(channels=[{channel_values}])"

class MSP_SET_RAW_RC_RESPONSE(MSP_MESSAGE):
    ID = 200
    _pack_ = 1