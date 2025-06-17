import socket
import serial
import time
import ctypes
import logging

from abc import ABC, abstractmethod
from concurrent.futures import ThreadPoolExecutor
from enum import Enum
from msp.messages.base_msg import MSP_MESSAGE
from msp.messages.enums import RESPONSES
from msp.tools.crc import crc8_dvb_s2
from msp.tools.logger import logger
from queue import Queue
from threading import Lock, Thread, Event

class ReadState(Enum):
    UNKNOWN = 0
    WAIT_FIRST_BYTE = 1
    WAIT_VERSION = 2
    WAIT_DIRECTION = 3

    WAIT_FLAG_V2 = 4
    WAIT_SIZE_V1 = 5

    WAIT_COMMAND_V2 = 6
    WAIT_COMMAND_V1 = 7

    WAIT_SIZE_V2 = 8

    WAIT_PAYLOAD = 9
    WAIT_CRC = 10


class CommInterface(ABC):
    stop_event_r = Event()
    stop_event_w = Event()
    stop_lock = Lock()
    __active_r = False
    __active_w = False

    def __init__(self):
        self.read_lock = Lock()
        self.write_lock = Lock()
        self.response_events_lock = Lock()

        self.disposed = False
        self.write_queue = Queue(maxsize=1000)

        self.cbs = {}
        self.__response_events = {}

        self.reader_state = ReadState.UNKNOWN

        self.callback_executor = ThreadPoolExecutor(max_workers=5, thread_name_prefix="msp_cb_executor")
        self.reader_th = Thread(target=self.read_loop, daemon=True, name="msp_reader")

        self.__writer_th = Thread(target=self.write_loop, daemon=True, name="msp_writer")
        self.__writer_th.start()

    @abstractmethod
    def connect(self):
        pass

    @abstractmethod
    def _read(self, size: int) -> bytes:
        pass

    @abstractmethod
    def _write(self, data: bytes):
        pass

    @abstractmethod
    def close(self):
        pass

    def active(self):
        return self.__active_r and self.__active_w

    def stop(self, max_timeout=10) -> bool:
        if not self.__active_r and not self.__active_w:
            return True

        logger.debug("Communication stopping...")
        with self.stop_lock:
            self.stop_event_r.set()
            self.stop_event_w.set()

            timeout = 0
            while (self.__active_w or self.__active_r) and timeout < max_timeout:
                time.sleep(0.1)
                timeout += 0.1

            if timeout >= max_timeout:
                raise Exception("Failed to stop MSP threads")

        self.stop_event_r.clear()
        self.stop_event_w.clear()

        logger.info("Communication stopped")
        return True

    def read_loop(self):
        self.__active_r = True
        crc_buffer = bytes()
        self.reader_state = ReadState.WAIT_FIRST_BYTE
        proto_v = -1
        target_sz = 0  # crc_buf + payload_sz

        while not self.stop_event_r.is_set():
            with self.read_lock:
                data = self._read(1)
            if data is None:
                continue

            match self.reader_state:
                case ReadState.WAIT_FIRST_BYTE:
                    if data == b"$":
                        self.reader_state = ReadState.WAIT_VERSION
                        crc_buffer = bytes()  # clear on start

                case ReadState.WAIT_VERSION:
                    if data == b"X" or data == b"M":
                        proto_v = 2 if data == b"X" else 1
                        self.reader_state = ReadState.WAIT_DIRECTION
                    else:
                        self.reader_state = ReadState.WAIT_FIRST_BYTE
                case ReadState.WAIT_DIRECTION:
                    if data == b">":
                        if proto_v == 2:
                            self.reader_state = ReadState.WAIT_FLAG_V2
                        elif proto_v == 1:
                            self.reader_state = ReadState.WAIT_SIZE_V1
                    else:
                        if data == b"<":
                            logger.warning("Reader: Wrong direction")
                        self.reader_state = ReadState.WAIT_FIRST_BYTE
                case ReadState.WAIT_SIZE_V1:
                    crc_buffer += data
                    self.reader_state = ReadState.WAIT_COMMAND_V1

                case ReadState.WAIT_FLAG_V2:
                    crc_buffer += data
                    self.reader_state = ReadState.WAIT_COMMAND_V2

                case ReadState.WAIT_COMMAND_V2:
                    crc_buffer += data
                    if len(crc_buffer) == 3:
                        self.reader_state = ReadState.WAIT_SIZE_V2

                case ReadState.WAIT_SIZE_V2:
                    crc_buffer += data
                    if len(crc_buffer) == 5:
                        self.reader_state = ReadState.WAIT_PAYLOAD
                        payload_sz = int.from_bytes(crc_buffer[-2:], "little")
                        target_sz = 5 + payload_sz

                        if payload_sz == 0:
                            self.reader_state = ReadState.WAIT_CRC
                        else:
                            self.reader_state = ReadState.WAIT_PAYLOAD

                case ReadState.WAIT_COMMAND_V1:
                    crc_buffer += data
                    payload_sz = crc_buffer[0]
                    target_sz = 3 + payload_sz
                    if payload_sz > 0:
                        self.reader_state = ReadState.WAIT_PAYLOAD
                    else:
                        self.reader_state = ReadState.WAIT_CRC

                case ReadState.WAIT_PAYLOAD:
                    crc_buffer += data
                    if len(crc_buffer) == target_sz:
                        self.reader_state = ReadState.WAIT_CRC

                case ReadState.WAIT_CRC:
                    self.reader_state = ReadState.WAIT_FIRST_BYTE
                    desired_crc = int.from_bytes(data, "little")

                    crc = 0
                    for byte in crc_buffer:
                        crc ^= byte
                        for _ in range(8):
                            crc = ((crc << 1) ^ 0xD5) if (crc & 0x80) else (crc << 1)
                        crc &= 0xFF
                    if desired_crc != crc:
                        logger.warning("Rreader: CRC MISSMATCH")
                        continue

                    if proto_v == 2:
                        cmd = int.from_bytes(crc_buffer[1:3], "little")
                        payload_sz = int.from_bytes(crc_buffer[3:5], "little")
                        payload = crc_buffer[5:]
                    elif proto_v == 1:
                        cmd = crc_buffer[1]
                        payload_sz = crc_buffer[0]
                        payload = crc_buffer[2:]

                    msg_cls = RESPONSES.get(cmd)
                    if msg_cls is None:
                        logger.warning(f"Received unknown message with id: {cmd}")
                        continue

                    cb = self.cbs.get(cmd)

                    with self.response_events_lock:
                        response_event = self.__response_events.get(cmd)

                        if cb is not None or response_event is not None:
                            msg_instance = msg_cls()
                            ctypes.memmove(ctypes.byref(msg_instance), payload, ctypes.sizeof(msg_instance))

                        if cb is not None:
                            self.callback_executor.submit(cb, msg_instance)

                        if response_event is not None:
                            self.__response_events[cmd]["d"] = msg_instance
                            response_event["e"].set()

        self.__active_r = False

    def write_loop(self):
        self.__active_w = True
        while not self.stop_event_w.is_set():
            try:
                data = self.write_queue.get(timeout=0.5)
                with self.write_lock:
                    self._write(data)
            except:
                continue
        self.__active_w = False

    def register_cb(self, id, cb):
        self.cbs.update({id: cb})

    def __mspv2_to_bytes(self, msg: MSP_MESSAGE):
        payload = bytes(msg)
        msg_id = msg.message_id()

        message = bytearray(b"$X<\x00" + msg_id.to_bytes(2, byteorder="little") + len(payload).to_bytes(2, byteorder="little") + payload)
        crc = 0
        for byte in message[3:]:
            crc = crc8_dvb_s2(crc, byte)
        message.append(crc)
        return message

    def send_msp_message(self, msg: MSP_MESSAGE):
        try:
            if not self.__active_w:
                return
            message = self.__mspv2_to_bytes(msg)
            self.write_queue.put(message, timeout=0.1)
        except Exception as e:
            pass

    def write_blocking(self, msg: MSP_MESSAGE):
        if not self.__active_w:
            return

        message = self.__mspv2_to_bytes(msg)
        with self.write_lock, self.read_lock:
            self._write(message)

    # TODO. fix synchronization
    # def send_and_wait(self, msg: MSP_MESSAGE, timeout=1.0):
    #     """
    #     blocking mode. Send MSP -> Receive Responce
    #     """
    #     if not self.active():
    #         return

    #     self.send_msp_message(msg)

    #     with self.response_events_lock:
    #         if msg.message_id() not in self.__response_events:
    #             self.__response_events.update({msg.message_id(): {"e": Event(), "d": None}})
    #     received = self.__response_events[msg.message_id()]["e"].wait(timeout)
    #     if not received:
    #         print("timeout receiving response")
    #     with self.response_events_lock:
    #         d = self.__response_events[msg.message_id()]["d"]
    #         self.__response_events.pop(msg.message_id())
    #     return d


class SerialComm(CommInterface):
    __reconnecting = True
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, timeout=1):
        self.port = port

        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        super().__init__()

    def __repr__(self):
        return f"SerialComm(port={self.port!r},baudrate={self.baudrate!r})"

    def connect(self):
        self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        if not self.reader_th.is_alive():
            self.reader_th.start()
        return True

    def _read(self, size: int) -> bytes:
        try:
            d = self.ser.read(size)
            return d
        except Exception as e:
            logger.error(f"Communication error: {e}")
            if not self.__reconnecting: self._try_reconnect()
            return None

    def _write(self, data: bytes):
        try:
            d = self.ser.write(data)
        except Exception as e:
            logger.error(f"Communication error: {e}")
            if not self.__reconnecting: self._try_reconnect()
            return None
        return d

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def _try_reconnect(self):
        self.__reconnecting = True
        logger.debug("Attempting to reconnect...")
        self.close()
        while self.active():
            try:
                with self.stop_lock:
                    time.sleep(1.0)
                    if self.active():
                        self.connect()
                logger.info("Reconnected")
                break
            except Exception as e:
                logger.error(f"Reconnect attempt failed: {e}")
            time.sleep(0.1)
        self.__reconnecting = False



class TCPComm(CommInterface):
    def __init__(self, host="127.0.0.1", port=5760):
        self.host = host
        self.port = port
        self.sock = None
        super().__init__()

    def __repr__(self):
        return f"TCPComm(host={self.host!r},port={self.port!r})"

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.host, self.port))
        self.sock.settimeout(1.0)  # Default timeout

        if not self.reader_th.is_alive():
            self.reader_th.start() # in case of reconnect

    def _read(self, size: int) -> bytes:
        try:
            d = self.sock.recv(size)
            return d
        except:
            return None

    def _write(self, data: bytes):
        try:
            self.sock.sendall(data)
        except Exception as e:
            logger.error(f"Communication error {e}")
            self._try_reconnect()
            raise  # Re-raise after reconnect attempt

    def close(self):
        if self.sock:
            self.sock.close()

    def _try_reconnect(self):
        logger.debug("Attempting to reconnect...")
        self.close()
        while self.active():
            try:
                with self.stop_lock:
                    time.sleep(1.0)
                    if self.active():
                        self.connect()
                logger.info("Reconnected")
                break
            except Exception as e:
                logger.error(f"Reconnect attempt failed: {e}")
            time.sleep(0.1)
