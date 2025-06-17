import ctypes
from typing import Any

class MSP_MESSAGE(ctypes.Structure):
    ID = 0
    _pack_ = 1

    def message_id(self) -> int:
        return self.ID

    def __init__(self, *args: Any, **kw: Any) -> None:
        super().__init__(*args, **kw)
        
    def copy(self):
        cls = type(self)
        copied = cls()
        ctypes.memmove(ctypes.byref(copied), ctypes.byref(self), ctypes.sizeof(self))
        return copied