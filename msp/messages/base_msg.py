import ctypes
from typing import Any

class MSP_MESSAGE(ctypes.Structure):
    ID = 0
    _pack_ = 1
    _no_reply = False


    def message_id(self) -> int:
        return self.ID

    def __init__(self, *args: Any, **kw: Any) -> None:
        super().__init__(*args, **kw)

    def set_no_reply(self, value: bool):
        self._no_reply = value

    def copy(self):
        cls = type(self)
        copied = cls()
        ctypes.memmove(ctypes.byref(copied), ctypes.byref(self), ctypes.sizeof(self))
        copied._no_reply = self._no_reply
        return copied