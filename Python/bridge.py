"""
Docstring for bridge
"""

from vehicle import System
from px4 import PX4
from signals import *
import threading

class Translator:
    """
    Docstring for Translator
    """
    @staticmethod
    def pack(sensor: HIL_SENSOR, 
             gps: HIL_GPS, rc_inputs: HIL_RC_INPUTS, 
             quat: HIL_STATE_QUAT, 
             heartbeat: HIL_HEARTBEAT, 
             time: HIL_SYSTEM_TIME, 
             flag: HIL_SEND_UPDATE_FLAG
             ) -> HIL_SEND:
        return HIL_SEND(
            sensor=sensor,
            gps=gps,
            rc_inputs=rc_inputs,
            quat=quat,
            heartbeat=heartbeat,
            time=time,
            flag=flag
        )

class Bridge:
    """
    Docstring for Converter
    """
    def __init__(self):
        """
        Docstring for __init__
        
        :param self: Description
        """
        # components #
        self.system: System = System()
        self.px4: PX4 = PX4()

        # sub components #
        self.heartbeat: HIL_HEARTBEAT = HIL_HEARTBEAT()
        self.time: HIL_SYSTEM_TIME = HIL_SYSTEM_TIME()

        # threading #
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

        # startup logic #
        self._startup: bool = True

    def _main_loop(self):
        """
        used to time sending and recieving packets to PX4
        """
        while not self._stop_event.is_set():
            if self._startup:
                # logic for sendng starting dummy data #
                pass
            else:
                # logic for sending actual data #
                pass

    def start(self):
        """
        Docstring for start
        
        :param self: Description
        """
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._main_loop, daemon=True)
        self._thread.start()
        print(f"DEBUG:: Starting bridge")

    def stop(self):
        """
        Docstring for stop
        
        :param self: Description
        """
        self._stop_event.set()
        if self._thread:
            self._thread.join()
        print(f"DEBUG:: Bridge stopped")