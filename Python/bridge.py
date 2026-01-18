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
    Docstring for Bridge
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
        self.send: HIL_SEND = HIL_SEND()
        self.rec: HIL_REC = HIL_REC()
    
        self.rc_inputs: HIL_RC_INPUTS = HIL_RC_INPUTS()
        self.time: HIL_SYSTEM_TIME = HIL_SYSTEM_TIME()
        self.sys_state: MAV_STATE = MAV_STATE.MAV_STATE_UNINIT

        # threading #
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

        # startup logic #
        self._startup: bool = True
    
    def _update_flag(self) -> HIL_SEND_UPDATE_FLAG:
        """
        Makes sure the messages within HIL_SEND get sent on time
        """

        # 4ms between HIL_SENSOR messages, 4e3 us
        # 8ms between HIL_STATE_QUATERNION messages, 8e3 us
        # 52ms between HIL_GPS messages, 5.2e4 us
        # 1000ms between each HEARTBEAT, 1e6 us
        # 4000ms between SYSTEM_TIME, 4e6 us

        # TODO, clean this up #
        SENSOR_US = 4E3
        QUAT_US = 8E3
        GPS_US = 5.2E4
        HEARTBEAT_US = 1E6
        TIME_US = 4E6

        # TODO #
        flag: HIL_SEND_UPDATE_FLAG = HIL_SEND_UPDATE_FLAG.NONE
        time_boot = self.time.time_boot_ms

        if not time_boot % SENSOR_US:
            flag |= HIL_SEND_UPDATE_FLAG.SENSOR

        if not time_boot % QUAT_US:
            flag |= HIL_SEND_UPDATE_FLAG.QUAT

        if not time_boot % GPS_US:
            flag |= HIL_SEND_UPDATE_FLAG.GPS

        if not time_boot % HEARTBEAT_US:
            flag |= HIL_SEND_UPDATE_FLAG.HEARTBEAT

        if not time_boot % TIME_US:
            flag |= HIL_SEND_UPDATE_FLAG.TIME

        return flag

    def _main_loop(self):
        """
        used to time sending and recieving packets to PX4
        """
        self.system.start()
        self.px4.start()
    
        while not self._stop_event.is_set():
            # update time #
            self.time = self.system.get_time()
            # update plant #
            self.sys_state = self.system.update(
                send=self.rec.actuator_controls
                )
            # recive from plant data (with noise from sensor classes) #
            self.send = Translator.pack(
                sensor=self.system.get_sensor(),
                gps=self.system.get_gps(),
                rc_inputs=self.rc_inputs,
                quat=self.system.get_quaternion(),
                heartbeat=self.system.get_heartbeat(),
                time=self.time,
                flag=self._update_flag()
            )
            # send to px4 #
            self.rec = self.px4.update(self.send)

            # Print debug #
            print(f"px4 heartbeat:: {self.rec.heartbeat}")
            print(f"plant heartbeat:: {self.system.get_heartbeat}")

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