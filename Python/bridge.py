"""
Docstring for bridge
"""

from signals import *
from pydantic import BaseModel, Field

class Converter:
    """
    Docstring for Converter
    """
    def _fill_sensor(self, )

    def from_vehicle(self, data: HIL_VEHICLE_STATE) -> HIL_SEND:
        """
        Docstring for from_plant
        
        :param self: Description
        :param data: Description
        :type data: tuple[STATES, DCM]
        """
        send: HIL_SEND = HIL_SEND()

        sensor = send.sensor
        gps = send.gps
        rc_inputs = send.rc_inputs
        quat = send.quat
        flag = send.flag
        heartbeat = send.heartbeat
        time = send.time

        return send

    def to_vehicle(self, data: HIL_REC) -> HIL_ACTUATOR_CTL:
        """
        Docstring for to_plant
        
        :param self: Description
        :param data: Description
        :type data: HIL_REC
        :return: Description
        :rtype: HIL_ACTUATOR_CTL
        """
        return data.actuator_controls
