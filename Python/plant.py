"""
Docstring for vehicle
"""
from signals import *
from pydantic import BaseModel, Field, PrivateAttr, validate_call
import threading
import socket
import time
import struct

class NetworkInterface(BaseModel):
    """
    Docstring for NetworkInterface
    """
    HOST: str = "127.0.0.1"
    SEND_PORT: int = 65432
    REC_PORT: int = 65431

    _send_socket: any = PrivateAttr(default=None)
    _rec_socket: any = PrivateAttr(default=None)

class UDPInterface(NetworkInterface, BaseModel):
    """
    Low-latency UDP interface for bidirectional plant model communication.
    
    Handles binary serialization of telemetry data using IEEE 754 
    double-precision floats.
    """
    _HANDSHAKE: list[float] = [1015.0]
     
    def send(self, BUFFER: list[float]) -> bool:
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

    def read(self, buffer_size: int, timeout_duration: float) -> list[float]:
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
            return list(unpacked_data)
            
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

class Translator:
    """
    Docstring for Translator
    """
    def _pack_bytes(y):
        """
        Docstring for _pack_bytes
        
        :param y: Description
        """
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