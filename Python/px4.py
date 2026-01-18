"""
Contains PX4 and MavlinkInterface Classes
"""
from signals import *
from pymavlink import mavutil
from enum import IntFlag, IntEnum
from pydantic import BaseModel, Field, PrivateAttr, validate_call
import threading
import time

#  Mavlink Wrapper #
class MavlinkInterface(BaseModel):
    """
    Handles the low-level MAVLink communication bridge for Hardware-In-The-Loop (HIL) simulation.

    This class manages the lifecycle of a MAVLink connection, including initializing 
    the transport (TCP/UDP), handshaking via heartbeats, and serializing/deserializing 
    MAVLink packets into Pydantic data models.

    Attributes:
        TYPE (str): Connection protocol type (e.g., 'tcpin', 'udpout', 'serial').
        HOST (str): The IP address or interface to bind/connect to.
        PORT (int): The network port for the MAVLink stream.
        _vehicle (any): Private storage for the underlying pymavlink connection object.
    """
    TYPE: str = "tcpin"
    HOST: str = "0.0.0.0"
    PORT: int = 4564

    _vehicle: any = PrivateAttr(default=None)

    def connect(self):
        """
        Initializes the MAVLink connection and performs an initial heartbeat handshake.

        Configures the connection string, instantiates the mavutil link, and blocks 
        until a heartbeat is received from the target system (e.g., PX4 or ArduPilot).

        Returns:
            mavutil.mavlink_connection: The active underlying connection object.
        
        Raises:
            Exception: If the connection cannot be established or the heartbeat times out.
        """
        connection_string = f"{self.TYPE}:{self.HOST}:{self.PORT}"
        self._vehicle = mavutil.mavlink_connection(connection_string)
        
        print(f"Connecting to {self.HOST}:{self.PORT}...")
        self._vehicle.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GENERIC,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )
        
        self._vehicle.wait_heartbeat()
        print(f"Connected to System {self._vehicle.target_system} "
              f"Component {self._vehicle.target_component}")

        return self._vehicle

    def send(self, send: HIL_SEND) -> None:
        """
        Dispatches simulation data to the autopilot based on active update flags.

        Evaluates the bitmask in `send.flag` to determine which MAVLink messages 
        (Sensor, GPS, RC, etc.) need to be packed and transmitted in the current frame.

        Args:
            send (HIL_SEND): A data container holding all current simulation states 
                             and the bitmask determining which fields to update.
        """
        if not send.flag:
            print(f"DEBUG:: Issue with send flag, {send.flag}")
            return

        # --- HIL SENSOR UPDATE ---
        if send.flag & HIL_SEND_UPDATE_FLAG.SENSOR:
            self._vehicle.mav.hil_sensor_send(
                send.sensor.time_usec, send.sensor.xacc, send.sensor.yacc, send.sensor.zacc,
                send.sensor.xgyro, send.sensor.ygyro, send.sensor.zgyro,
                send.sensor.xmag, send.sensor.ymag, send.sensor.zmag,
                send.sensor.abs_pressure, send.sensor.diff_pressure, send.sensor.pressure_alt,
                send.sensor.temp, send.sensor.fields_updated, 0
            )

        # --- HIL GPS UPDATE ---
        if send.flag & HIL_SEND_UPDATE_FLAG.GPS:
            self._vehicle.mav.hil_gps_send(
                send.gps.time_usec, send.gps.fix_type, send.gps.lat_degE7, send.gps.lon_degE7,
                send.gps.alt_mm, send.gps.eph, send.gps.epv, send.gps.vel_cms,
                send.gps.vn_cms, send.gps.ve_cms, send.gps.vd_cms, send.gps.cog,
                send.gps.satellites_visible, 0, 0
            )

        # --- HIL RC INPUTS UPDATE ---
        if send.flag & HIL_SEND_UPDATE_FLAG.RC:
            self._vehicle.mav.hil_rc_inputs_raw_send(
                send.rc_inputs.time_usec,
                send.rc_inputs.chan1_raw, send.rc_inputs.chan2_raw, 
                send.rc_inputs.chan3_raw, send.rc_inputs.chan4_raw,
                send.rc_inputs.chan5_raw, send.rc_inputs.chan6_raw,
                send.rc_inputs.chan7_raw, send.rc_inputs.chan8_raw,
                send.rc_inputs.chan9_raw, send.rc_inputs.chan10_raw,
                send.rc_inputs.chan11_raw, send.rc_inputs.chan12_raw,
                send.rc_inputs.rssi
            )

        # --- HIL QUATERNION UPDATE ---
        if send.flag & HIL_SEND_UPDATE_FLAG.QUAT:
            self._vehicle.mav.hil_state_quaternion_send(
                send.quat.time_usec, send.quat.attitude_quaternion,
                send.quat.rollspeed_rads, send.quat.pitchspeed_rads, send.quat.yawspeed_rads,
                send.quat.lat_degE7, send.quat.lon_degE7, send.quat.alt_mm,
                send.quat.vx_cms, send.quat.vy_cms, send.quat.vz_cms,
                send.quat.ind_airspeed_cms, send.quat.true_airspeed_cms,
                send.quat.xacc_mG, send.quat.yacc_mG, send.quat.zacc_mG
            )

        # --- SYSTEM TIME UPDATE ---
        if send.flag & HIL_SEND_UPDATE_FLAG.TIME:
            self._vehicle.mav.system_time_send(
                send.time.time_unix_usec, send.time.time_boot_ms
            )

        # --- HEARTBEAT UPDATE ---
        if send.flag & HIL_SEND_UPDATE_FLAG.HEARTBEAT:
            self._vehicle.mav.heartbeat_send(
                send.heartbeat.type, send.heartbeat.autopilot,
                send.heartbeat.base_mode, send.heartbeat.custom_mode,
                send.heartbeat.system_status, send.heartbeat.mavlink_version
            )

    def recieve(self) -> HIL_REC:
        """
        Polls the MAVLink buffer for incoming messages from the autopilot.

        Checks for a single pending message in the transport buffer. If a recognized 
        HIL message (Actuators, Heartbeat, or System Time) is found, it is parsed 
        into the appropriate Pydantic model and returned.

        Returns:
            HIL_REC: A container object populated with the received message data.
            None: If no message was available in the buffer.
        """
        msg = self._vehicle.recv_match(blocking=True, timeout=0.01)

        if not msg:
            return None
        
        rec = HIL_REC()
        rec_type = msg.get_type()

        if rec_type == 'HIL_ACTUATOR_CONTROLS':
            rec.actuator_controls = HIL_ACTUATOR_CTL(**msg.to_dict())
        elif rec_type == 'HEARTBEAT':
            rec.heartbeat = HIL_HEARTBEAT(**msg.to_dict())
        elif rec_type == 'SYSTEM_TIME':
            rec.time = HIL_SYSTEM_TIME(**msg.to_dict())

        return rec

# PX4 Wrapper #
class PX4:            
    """
    High-level controller for PX4 Autopilot interaction.
    """
    _TYPE: str = "tcpin"
    _HOST: str = "0.0.0.0"
    _PORT: int = 4564
    
    # Standard functions #
    def __init__(self, connection_type: str = _TYPE, host: str = _HOST, port: int = _PORT):
        """
        Initializes the PX4 controller and state containers.

        Args:
            connection_type: MAVLink protocol (e.g., 'tcpin', 'udpout').
            host: IP address of the autopilot or simulation host.
            port: Network port for the connection.
        """
        self.interface = MavlinkInterface(TYPE = connection_type, 
                                          HOST = host, 
                                          PORT = port)
        self.state = HIL_REC()

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._receiver_thread: threading.Thread | None = None
        
    def __str__(self):
        status = "CONNECTED" if self._receiver_thread and self._receiver_thread.is_alive() else "DISCONNECTED"
        return f"<PX4 Controller: {self.interface.HOST}:{self.interface.PORT} [{status}]>"
        
    def __repr__(self):
        return self.__str__()

    def _update_loop(self):
        """
        Internal loop running in a background thread.
        Constantly drains the MAVLink buffer and updates the 'state' shadow.
        """
        while not self._stop_event.is_set():
            new_data = self.interface.recieve()
            ''' no sleep needed, timeout included on recieve() '''

            if new_data:
                with self._lock:
                    if new_data.actuator_controls:
                        self.state.actuator_controls = new_data.actuator_controls
                    if new_data.heartbeat:
                        self.state.heartbeat = new_data.heartbeat
                    if new_data.time:
                        self.state.time = new_data.time
            else:
                print(f"DEBUG:: Didn't recieve new data, returned {new_data}")

    def start(self):
        """
        Establishes connection and launches the background telemetry thread.
        """
        try:
            self.interface.connect()
            self._stop_event.clear()
            self._receiver_thread = threading.Thread(target=self._update_loop, daemon=True)
            self._receiver_thread.start()
        except Exception as e:
            print(f"Failed to start PX4 interface: {e}")
            raise

    def stop(self):
            """
            Signals the background thread to exit and cleans up resources.
            """
            self._stop_event.set()
            if self._receiver_thread:
                self._receiver_thread.join(timeout=1.0)

    # funcions used to send and recieve data from PX4 #
    @validate_call
    def update(self, sent: HIL_SEND) -> HIL_REC:
        """
        Sends simulated sensor data to the PX4 flight stack.

        Args:
            sent: The HIL_SEND container with active update flags.
        Returns:
            HIL_REC: A snapshot of the current actuators and status.
        """
        self.interface.send(sent)

        with self._lock:
            return self.state.model_copy()

    # def get_outputs(self) -> HIL_REC:
    #     """
    #     Provides thread-safe access to the latest autopilot state.

    #     Returns:
    #         HIL_REC: A snapshot of the current actuators and status.
    #     """
    #     with self._lock:
    #         return self.state.model_copy()

# main, for testing #
def main():
    TYPE: str = "tcpin"
    HOST: str = "0.0.0.0"
    PORT: int = 4564
    
    p = PX4(TYPE, HOST, PORT)

    try:
        # 1. Start the connection and the background thread
        p.start()
        print("Interface started. Waiting for data...")

        # 2. Keep the main thread alive in a loop
        while True:
            # Check the state every second
            state = p.get_outputs()
            
            if state.heartbeat:
                print(f"Heartbeat received! Mode: {state.heartbeat.base_mode}")
            else:
                print("Waiting for heartbeat...")
                
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nStopping simulation...")
        p.stop()

if __name__ == "__main__":
    main()