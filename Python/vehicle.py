"""
Docstring for vehicle
"""
from signals import *
from units import *
from pydantic import PrivateAttr, validate_call
import socket
import struct
import threading
import time
import utm
import geopy.distance

class Packet(BaseModel):
    """
    Docstring for data
    """
    data_string: list[float]

    def __len__(self):
        return len(self.data_string)

    def __iter__(self):
        return iter(self.data_string)
    
    def __getitem__(self, index):
        return self.data_string[index]

# TODO generalize with these classes 
class Command(Packet):
    data_string: list[float] = Field(
        default_factory=lambda: [0.0] * NUM_ACTUATORS,
        min_length=NUM_ACTUATORS,
        max_length=NUM_ACTUATORS
    )

# TODO generalize with these classes
class Telemetry(Packet):
    data_string: list[float] = Field(
        default_factory=lambda: [0.0] * NUM_PLANT_OUTPUTS,
        min_length=NUM_PLANT_OUTPUTS,
        max_length=NUM_PLANT_OUTPUTS
    )

class NetworkInterface(BaseModel):
    """
    Docstring for NetworkInterface
    """
    HOST: str = "127.0.0.1"
    SEND_PORT: int = 65432
    REC_PORT: int = 65431

    _send_socket: any = PrivateAttr(default=None)
    _rec_socket: any = PrivateAttr(default=None)

class UDPInterface(NetworkInterface):
    """
    Low-latency UDP interface for bidirectional plant model communication.
    
    Handles binary serialization of telemetry data using IEEE 754 
    double-precision floats.
    """
    _HANDSHAKE: Packet = Packet(data_string=[1015.0] + [0.0]*(NUM_ACTUATORS-1))
    _ENDSEQUENCE: Packet = Packet(data_string=[67]*NUM_ACTUATORS)

    def send(self, BUFFER: Packet) -> bool:
        """
        Packs and transmits a list of floats as Little-Endian doubles.

        Args:
            BUFFER: List of values to be serialized and sent.
            
        Returns:
            bool: True if transmission was successful, False on socket error.
        """
        send_bytes = struct.pack(f'<{len(BUFFER)}d', *BUFFER)
        try:
            sent = self._send_socket.sendto(send_bytes, (self.HOST, self.SEND_PORT))
            return True if sent else False
        except socket.error as e:
            print(f"UDP Send error: {e}")
            return False

    def read(self, buffer_size: int = 1024, timeout_duration: float = 2.0) -> Packet:
        """
        Receives and unpacks a UDP packet into a list of floats.

        Args:
            buffer_size: Max bytes to read from the buffer.
            timeout_duration: Seconds to wait before timing out.

        Returns:
            list[float]: Unpacked telemetry data. Returns [NaN] on failure.
        """
        self._rec_socket.settimeout(timeout_duration)
        try:
            rec_bytes, addr = self._rec_socket.recvfrom(buffer_size)
            # Determine count based on 8-byte double width
            num_doubles = len(rec_bytes) // 8
            unpacked_data = struct.unpack(f'<{num_doubles}d', rec_bytes)
            return Packet(data_string=unpacked_data)
            
        except socket.timeout:
            print(f"UDP Timeout Error")
            return [float('nan')]
        except struct.error as e:
            print(f"UDP Packing error: {e}")
            return [float('nan')]

    def connect(self, timeout_limit: int) -> bool:
        """
        Synchronizes with the plant model via a verification handshake.

        Iteratively pings the plant and validates the returned ID. 
        Continues until the handshake ID is confirmed or the retry 
        limit is exceeded.

        Args:
            timeout_limit: Maximum number of failed attempts before aborting.

        Returns:
            bool: True if connection verified, False on failure.
        """
        self._send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rec_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        try:
            self._rec_socket.bind(("0.0.0.0", self.REC_PORT))
        except socket.error as e:
            print(f"UDP failed to bind: {e}")
            return False

        rec = 0
        timeout_count = 0
        while rec != self._HANDSHAKE[0]:
            self.send(self._HANDSHAKE)
            response = self.read(1024, 2.0)
            rec = response[0]

            # NaN check: if response was a timeout, increment counter
            if rec != rec: 
                timeout_count += 1
                print(f"UDP:: Connection attempt {timeout_count} timed out...")
            elif rec != self._HANDSHAKE[0]:
                print(f"UDP:: Incorrect handshake recieved")
            
            if timeout_count >= timeout_limit:
                print(f"UDP:: Error: No plant response after {timeout_limit} attempts.")
                return False

        print("UDP:: Handshake Verified!")
        # stop handshake #
        self.send([0.0]*NUM_ACTUATORS)
        return True

    def disconnect(self):
        self.send(self._ENDSEQUENCE)
        self._send_socket.close()
        self._rec_socket.close()

class Translator: # TODO, improve with iterator and hydration
    """
    Docstring for Translator
    """
    @staticmethod
    def pack(actuator_controls: HIL_ACTUATOR_CTL) -> Packet:
        return Packet(data_string=actuator_controls.controls)

    @staticmethod
    def unpack(message: Packet) -> tuple[STATES, DCM]:
        """
        Unpacks a flat list of floats into structured STATES and DCM objects,
        skipping time_usec as it is not present in the raw data stream.
        """
        states: STATES = STATES()
        dcm: DCM = DCM()
        
        # Calculate size based on your model definitions
        num_states = STATES.get_num_states()
        raw_data = message.data_string
        
        # Check recieved data length, expect 33 #
        if len(raw_data) == STATES.get_num_states() + DCM.get_num_members():
            # Split raw data: States first, DCM follows
            state_data = raw_data[:num_states]
            dcm_data = raw_data[num_states:]

            # --- Map EF_velo (Indices 0-2) ---
            states.earth_frame_v.Vxe = state_data[0]
            states.earth_frame_v.Vye = state_data[1]
            states.earth_frame_v.Vze = state_data[2]

            # --- Map EF_pos (Indices 3-5) ---
            states.earth_frame_x.Xe = state_data[3]
            states.earth_frame_x.Ye = state_data[4]
            states.earth_frame_x.Ze = state_data[5]

            # --- Map EF_acc (Indices 6-8) ---
            states.earth_frame_a.Axe = state_data[6]
            states.earth_frame_a.Aye = state_data[7]
            states.earth_frame_a.Aze = state_data[8]

            # --- Map EULER_ANGLES (Indices 9-11) ---
            states.euler_angles.Phi   = state_data[9]
            states.euler_angles.Theta = state_data[10]
            states.euler_angles.Psi   = state_data[11]

            # --- Map BF_velo (Indices 12-14) ---
            states.body_frame_v.U = state_data[12]
            states.body_frame_v.V = state_data[13]
            states.body_frame_v.W = state_data[14]

            # --- Map BF_ang_rate (Indices 15-17) ---
            states.body_frame_w.p = state_data[15]
            states.body_frame_w.q = state_data[16]
            states.body_frame_w.r = state_data[17]

            # --- Map BF_acc (Indices 18-20) ---
            states.body_frame_a.U_dot = state_data[18]
            states.body_frame_a.V_dot = state_data[19]
            states.body_frame_a.W_dot = state_data[20]

            # --- Map BF_ang_acc (Indices 21-23) ---
            states.body_frame_al.p_dot = state_data[21]
            states.body_frame_al.q_dot = state_data[22]
            states.body_frame_al.r_dot = state_data[23]

            # TODO, make the rest look like this, include it in the DCM and STATE classes
            # --- Map DCM (Remaining Indices) ---
            dcm_fields = list(DCM.model_fields.keys())
            for i, val in enumerate(dcm_data):
                if i < len(dcm_fields):
                    setattr(dcm, dcm_fields[i], val)
        else: # Incorrect data sent #
            print(f"DEBUG:: Unexpected data recieved, {raw_data} of length: {len(raw_data)}")

        return states, dcm

class Plant:
    """
    Docstring for Plant
    """
    def __init__(self):
        self.STATES: STATES = STATES()
        self.DCM: DCM = DCM()

        self.UDP: UDPInterface = UDPInterface()
        self.connected = False

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._receiver_thread: threading.Thread | None = None

    def __str__(self):
        status = "CONNECTED" if self.connected and self._receiver_thread and self._receiver_thread.is_alive() else "DISCONNECTED"
        return f"<Plant: {self.UDP.HOST}:{self.UDP.REC_PORT} [{status}]>"
        
    def __repr__(self):
        return self.__str__()

    def _update_loop(self):
        while not self._stop_event.is_set():
            raw_data = self.UDP.read()

            if raw_data == raw_data:
                new_states, new_dcm = Translator.unpack(raw_data)

                with self._lock:
                    self.STATES = new_states
                    self.DCM = new_dcm
            else:
                print(f"DEBUG:: Could not read from Plant, returned {raw_data}")
        # # maybe add:def _update_loop(self):
        # while not self._stop_event.is_set():
        #     # 1. Block for the first available packet
        #     raw_data = self.UDP.read()
            
        #     # 2. "Drain" logic: Check if there are more packets waiting 
        #     # and skip to the very last one.
        #     while True:
        #         try:
        #             # Set a tiny timeout to check for pending packets
        #             self.UDP._rec_socket.settimeout(0) 
        #             more_data = self.UDP.read()
        #             if more_data:
        #                 raw_data = more_data
        #         except (BlockingIOError, socket.timeout):
        #             break # No more packets waiting
            
        #     # 3. Process ONLY the newest raw_data
        #     if raw_data:
        #         # ... rest of your unpack and lock logic ...

    def start(self) -> bool:
        timeout_limit: int = 10
        self.connected = self.UDP.connect(timeout_limit)

        if self.connected:
            self._stop_event.clear()
            self._receiver_thread = threading.Thread(target=self._update_loop, daemon=True)
            self._receiver_thread.start()
            print("Plant: Update loop started.")
        else:
            print(f"DEBUG:: Plant failed to connect")

        return self.connected

    def stop(self):
        self._stop_event.set()
        if self._receiver_thread:
            self._receiver_thread.join()
        self.UDP.disconnect()

    def get_data(self, send: HIL_ACTUATOR_CTL) -> tuple[STATES, DCM]:
        if self.UDP.send(Translator.pack(send)):
            print(f"DEBUG:: Sent {send}")
        else:
            print(f"DEBUG:: Failed to send {send}")

        with self._lock:
            return(
                self.STATES,
                self.DCM
            )

class Vehicle:
    """
    Docstring for Vehicle
    """
    def __init__(self):
        self.plant: Plant = Plant()
        self.states: STATES = STATES()
        self.dcm: DCM = DCM()
        self.pos: VEHICLE_POSITION = VEHICLE_POSITION()
        self.env: VEHICLE_ENVIORNMENT = VEHICLE_ENVIORNMENT()

        self._start_lat_degE7 = 400740840
        self._start_lon_degE7 = -830776070
        self._start_alt_mm = 267000
        self._start_temp_degC = 6
        self._start_pressure_inHg = 30

    def vehicle_start(self) -> bool:
        self.pos.altitude.alt_mm = self._start_alt_mm
        self.pos.coordinates.lat_degE7 = self._start_lat_degE7
        self.pos.coordinates.lon_degE7 = self._start_lon_degE7
        self.env.temp_C = self._start_temp_degC
        self.env.pressure_inHg = self._start_pressure_inHg

        return self.plant.start()

    def vehicle_stop(self):
        self.plant.stop()

    def _update_position(self):
        ns_delta = self.states.earth_frame_x.Xe
        ew_delta = self.states.earth_frame_x.Ye
        
        dist = math.sqrt(ns_delta**2 + ew_delta**2)
        track = math.degrees(math.atan2(ew_delta, ns_delta))

        point = geopy.distance.distance(
            meters=dist
            ).destination(
            (self._start_lat_degE7 / 1e7,
             self._start_lon_degE7 / 1e7),
             bearing=track
        )
        self.pos.coordinates.lat_degE7 = int(point.latitude * 1e7)
        self.pos.coordinates.lon_degE7 = int(point.longitude * 1e7)

    def _update_altitude(self):
        # update altitude, Ze (m) altitude (mm)
        self.pos.altitude = self._start_alt_mm - self.states.earth_frame_x.Ze * 1e3

    def _update_enviornment(self):
        # approx 1 in hg per 1000 ft #
        self.env.pressure_inHg = self._start_pressure_inHg - self.states.earth_frame_x.Ze * 3.281 / 1000
        # approx 6.5deg C change per 1000 meters #
        self.env.temp_C = self._start_temp_degC + self.states.earth_frame_x.Ze * 6.5 / 1000 

    def get_vehicle_state(self, send: HIL_ACTUATOR_CTL) -> HIL_VEHICLE_STATE:
        self.states, self.dcm = self.plant.get_data(send)

        self._update_position()
        self._update_altitude()
        self._update_enviornment()

        return HIL_VEHICLE_STATE(
            states=self.states,
            dcm=self.dcm,
            pos=self.pos,
            env=self.env
        )
    
class Sensor(BaseModel):
    sensor_data: Packet
    updated: HIL_SENSOR_UPDATE_FLAG = HIL_SENSOR_UPDATE_FLAG.RESET

    def update(self, state: HIL_VEHICLE_STATE) -> HIL_SENSOR_UPDATE_FLAG:
        """
        Base update method. 
        Subclasses should override this with specific logic.
        """
        pass

    def read(self) -> Packet:
        return self.sensor_data

class Magnatometer(Sensor):
    @property
    def x(self): return self.sensor_data[0]
    @property
    def y(self): return self.sensor_data[1]
    @property
    def z(self): return self.sensor_data[2]

    def update(self, vehicle_data: HIL_VEHICLE_STATE):
        self.updated = 0
        self.new_data = [
            # Somehow get magnetic field from vehicle state
        ]
        if self.sensor_data[self.x] == self.new_data[self.x]:
            updated += HIL_SENSOR_UPDATE_FLAG.XMAG
        if self.sensor_data[self.y] == self.new_data[self.y]:
            updated += HIL_SENSOR_UPDATE_FLAG.YMAG
        if self.sensor_data[self.z] == self.new_data[self.z]:
            updated += HIL_SENSOR_UPDATE_FLAG.ZMAG

        self.sensor_data = self.new_data
        
        return updated

class Accelerometer(Sensor):
    def update(self, vehicle_data: HIL_VEHICLE_STATE):
        # Add noise here #
        self.new_data = [
            vehicle_data.states.body_frame_a.U_dot,
            vehicle_data.states.body_frame_a.V_dot,
            vehicle_data.states.body_frame_a.W_dot
        ]
        if self.sensor_data != self.new_data:
            self.sensor_data = self.new_data
            return True
        return False
    @property
    def x(self): return self.sensor_data[0]
    @property
    def y(self): return self.sensor_data[1]
    @property
    def z(self): return self.sensor_data[2]

class Gyro(Sensor):
    def update(self, vehicle_data: HIL_VEHICLE_STATE):
        self.new_data = [
            vehicle_data.states.body_frame_al.p_dot,
            vehicle_data.states.body_frame_al.q_dot,
            vehicle_data.states.body_frame_al.r_dot
        ]
        if self.sensor_data != self.new_data:
            self.sensor_data = self.new_data
            return True
        return False
    @property
    def x(self): return self.sensor_data[0]
    @property
    def y(self): return self.sensor_data[1]
    @property
    def z(self): return self.sensor_data[2]

class Barometer(Sensor):
    def update(self, vehicle_data: HIL_VEHICLE_STATE):
        self.new_data = [
            vehicle_data.env.pressure_inHg,
            vehicle_data.pos.altitude.alt_mm / 1e3, # TODO, confrm SI units
            vehicle_data.env.temp_C
        ]
        if self.sensor_data != self.new_data:
            self.sensor_data = self.new_data
            return True
        return False
    @property
    def abs_pressure(self): return self.sensor_data[0]
    @property
    def pressure_alt(self): return self.sensor_data[1]
    @property
    def temp(self): return self.sensor_data[2]

class Airspeed(Sensor):


    @property
    def diff_pressure(self): return self.sensor_data[0]

    def update(self, vehicle_data: HIL_VEHICLE_STATE):
        airspeed = vehicle_data.states.body_frame_v
        self.new_data = [
            # TODO, use DCM to determine track and thus airspeed, then derive pressure difference
        ]
        if self.sensor_data != self.new_data:
            self.sensor_data = self.new_data
            return True
        return False

class Gps(BaseModel):
    @property
    def fix_type(self): return self.sensor_data[0]
    @property
    def lat(self): return self.sensor_data[1]
    @property
    def lon(self): return self.sensor_data[2]
    @property
    def alt(self): return self.sensor_data[3]
    @property
    def eph(self): return self.sensor_data[4]
    @property
    def epv(self): return self.sensor_data[5]
    @property
    def vel(self): return self.sensor_data[6]
    @property
    def vn(self): return self.sensor_data[7]
    @property
    def ve(self): return self.sensor_data[8]
    @property
    def vd(self): return self.sensor_data[9]
    @property
    def cog(self): return self.sensor_data[10]
    @property
    def sats_vis(self): return self.sensor_data[11]

    def update(self, vehicle_data: HIL_VEHICLE_STATE):
        self.new_data = [
            # TODO, write logic to assign gps values from states
        ]

class System:
    """
    Docstring for System
    """
    class SENSORS(IntEnum):
        MAG = 0
        ACCEL = 1
        GYRO = 2
        BARO = 3
        AIRSPD = 4

    def __init__(self):
        self.vehicle: Vehicle = Vehicle()
        self.sensors: list[Sensor] = [
            Magnatometer(),
            Accelerometer(),
            Gyro(),
            Barometer(),
            Airspeed()
        ]
        self.gps = Gps()

        self.flag: HIL_SENSOR_UPDATE_FLAG = HIL_SENSOR_UPDATE_FLAG.RESET
        self.internal_time: HIL_SYSTEM_TIME = HIL_SYSTEM_TIME()
        self.state: MAV_STATE = MAV_STATE.MAV_STATE_BOOT

    def start(self) -> bool:
        started = self.vehicle.vehicle_start()
        if started:
            self.state = MAV_STATE.MAV_STATE_STANDBY
            self.flag = 0
            self._start_time: int = int(time.time() * 1e-6)
        else:
            self.state = MAV_STATE.MAV_STATE_CRITICAL

        return started

    def stop(self):
        self.state = MAV_STATE.MAV_STATE_POWEROFF
        self.vehicle.vehicle_stop()
        self.state = MAV_STATE.MAV_STATE_FLIGHT_TERMINATION
    
    def update(self, send: HIL_ACTUATOR_CTL):
        """
        Main update function, gets new data from vehicle, updates sensors
        update time
        
        :param self: Description
        """
        # update time #
        current_time = int(time.time * 1e-6)
        self.internal_time.time_unix_usec = current_time
        self.internal_time.time_boot_ms = current_time - self._start_time

        # update plant #
        state = self.vehicle.get_vehicle_state(send)

        # update sensors #
        for sensor in self.sensors:
            flag += sensor.update(state)

    def get_sensor(self) -> HIL_SENSOR:
        """
        returns updated sensor data in mavlnk format
        
        :param self: Description
        :return: Description
        :rtype: HIL_SENSOR
        """
        accel = self.sensors[self.SENSORS.ACCEL]
        gyro = self.sensors[self.SENSORS.GYRO]
        mag = self.sensors[self.SENSORS.MAG]
        baro = self.sensors[self.SENSORS.BARO]
        airspd = self.sensors[self.SENSORS.AIRSPD]

        return HIL_SENSOR(
            time_usec=self.internal_time.time_boot_ms,

            xacc=accel.x,
            yacc=accel.y,
            zacc=accel.z,

            xgyro=gyro.x,
            ygyro=gyro.y,
            zgyro=gyro.z,

            xmag=mag.x,
            ymag=mag.y,
            zmag=mag.z,

            abs_pressure=baro.abs_pressure,
            diff_pressure=airspd.diff_pressure,
            pressure_alt=baro.pressure_alt,
            temp=baro.temp,

            fields_updated=self.flag
        )

    def get_gps(self) -> HIL_GPS:
        """
        returns gps data in mavlink format
        
        :param self: Description
        :return: Description
        :rtype: HIL_GPS
        """
        pass

    def get_quaternion(self) -> HIL_STATE_QUAT:
        """
        returns quaternion data in mavlink format
        
        :param self: Description
        :return: Description
        :rtype: HIL_STATE_QUAT
        """
        pass

    def get_time(self) -> HIL_SYSTEM_TIME:
        """
        returns time data in mavlink format
        
        :param self: Description
        :return: Description
        :rtype: HIL_SYSTEM_TIME
        """
        pass


# main, for testing #
def main():
    vehicle = Vehicle()

    if vehicle.vehicle_start():
        vehicle.vehicle_stop()

if __name__ == "__main__":
    main()