"""
Docstring for vehicle
"""
from signals import *
from pydantic import BaseModel, Field, PrivateAttr, validate_call
import socket
import struct

class Message(BaseModel):
    """
    Docstring for data
    """
    data_string: list[float]

    def __len__(self):
        return len(self.data_string)

    def __iter__(self):
        return iter(self.data_string)
    
    def __iter__(self):
        return iter(self.data_string)
    
    def __getitem__(self, index):
        return self.data_string[index]
    
class Command(Message):
    data_string: list[float] = Field(
        default_factory=lambda: [0.0] * NUM_ACTUATORS,
        min_length=NUM_ACTUATORS,
        max_length=NUM_ACTUATORS
    )

# TODO generalize with these classes
class Telemetry(Message):
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
    _HANDSHAKE: Message = Message(data_string=[1015.0] + [0.0]*15)

    def send(self, BUFFER: Message) -> bool:
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

    def read(self, buffer_size: int, timeout_duration: float) -> Message:
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
            return Message(data_string=unpacked_data)
            
        except socket.timeout:
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
        return True

class Translator: # TODO, improve wih iterator and hydration
    """
    Docstring for Translator
    """
    @staticmethod
    def pack(actuator_controls: HIL_ACTUATOR_CTL) -> Message:
        return Message(data_string=actuator_controls.controls)

    @staticmethod
    def unpack(message: Message) -> tuple[STATES, DCM]:
        """
        Unpacks a flat list of floats into structured STATES and DCM objects,
        skipping time_usec as it is not present in the raw data stream.
        """
        states: STATES = STATES()
        dcm: DCM = DCM()
        
        # Calculate size based on your model definitions
        num_states = STATES.get_num_states()
        raw_data = message.data_string
        
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

        return states, dcm

        pass

class Vehicle:
    """
    Docstring for Plant
    """
    pass

# main, for testing #
def main():
    face: UDPInterface = UDPInterface()
    connected = face.connect(60)

    if connected:
        print("Connecetd!")
    else:
        print("failed")
    pass

if __name__ == "__main__":
    main()