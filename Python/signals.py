"""
MAVLink HIL (Hardware-In-The-Loop) Data Interface.

Provides porotocls used to connect Plant, PX4 and Bridge

"""

from enum import IntFlag, IntEnum
from pydantic import BaseModel, Field


# Constants #


UINT16_MAX = 65535  # Maximum value for 16-bit unsigned integers
UINT8_MAX = 255     # Maximum value for 8-bit unsigned integers

# Implimentation specific constants, TODO: Make this more general
NUM_ACTUATORS = 16
NUM_PLANT_OUTPUTS = 33


# Enums and FLags #


class HIL_SEND_UPDATE_FLAG(IntFlag):
    """
    Bitmask indicating which mavlink message type needs to be sent
    """
    NONE = 0
    SENSOR = 1 << 0
    GPS = 1 << 1
    RC = 1 << 2
    QUAT = 1 << 3
    TIME = 1 << 4
    HEARTBEAT = 1 << 5

class HIL_REC_UPDATE_FLAG(IntFlag):
    """
    Bitmask indicating which mavlink message type was recieved
    """
    NONE = 0
    CONTROLS = 1 << 0
    HEARTBEAT = 1 << 1
    TIME = 1 << 2

class HIL_SENSOR_UPDATE_FLAG(IntFlag):
    """
    Bitmask indicating which fields in the HIL_SENSOR message have been updated.
    
    Ref: https://mavlink.io/en/messages/common.html#HIL_SENSOR
    """
    XACC = 1
    YACC = 2
    ZACC = 4
    XGYRO = 8
    YGYRO = 16
    ZGYRO = 32
    XMAG = 64
    YMAG = 128
    ZMAG = 256
    ABS_PRESSURE = 512
    DIFF_PRESSURE = 1024
    PRESSURE_ALT = 2048
    TEMPERATURE = 4096
    RESET = 2147483648  # Bit 31: Full sensor suite reset

class GPS_FIX_TYPE(IntEnum):
    """
    Type of GPS fix as defined in the MAVLink standard.
    """
    NO_GPS = 0
    NO_FIX = 1
    FIX_2D = 2
    FIX_3D = 3
    DGPS = 4
    RTK_FLOAT = 5
    RTK_FIXED = 6
    STATIC = 7
    PPP = 8

class MAV_AUTOPILOT(IntEnum):
    """
    MAVLink Autopilot Types (MAV_AUTOPILOT)
    Exact MAVLink names for values 0 through 20.
    """
    MAV_AUTOPILOT_GENERIC = 0
    MAV_AUTOPILOT_RESERVED = 1
    MAV_AUTOPILOT_SLUGS = 2
    MAV_AUTOPILOT_ARDUPILOTMEGA = 3
    MAV_AUTOPILOT_OPENPILOT = 4
    MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY = 5
    MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY = 6
    MAV_AUTOPILOT_PROXIMITY = 7
    MAV_AUTOPILOT_SENSORS_ONLY = 8
    MAV_AUTOPILOT_UDB = 9
    MAV_AUTOPILOT_FP = 10
    MAV_AUTOPILOT_PX4 = 12
    MAV_AUTOPILOT_SMACCMPILOT = 13
    MAV_AUTOPILOT_AUTOQUAD = 14
    MAV_AUTOPILOT_ARMAZILA = 15
    MAV_AUTOPILOT_AEROB = 16
    MAV_AUTOPILOT_ASLUAV = 17
    MAV_AUTOPILOT_SMARTAP = 18
    MAV_AUTOPILOT_AIRRAILS = 19
    MAV_AUTOPILOT_REFLEX = 20

class MAV_TYPE(IntEnum):
    """
    Full MAVLink Vehicle Types (MAV_TYPE)
    Includes standard, VTOL, and specialized craft.
    """
    MAV_TYPE_GENERIC = 0
    MAV_TYPE_FIXED_WING = 1
    MAV_TYPE_QUADROTOR = 2
    MAV_TYPE_COAXIAL = 3
    MAV_TYPE_HELICOPTER = 4
    MAV_TYPE_ANTENNA_TRACKER = 5
    MAV_TYPE_GCS = 6
    MAV_TYPE_AIRSHIP = 7
    MAV_TYPE_FREE_BALLOON = 8
    MAV_TYPE_ROCKET = 9
    MAV_TYPE_GROUND_ROVER = 10
    MAV_TYPE_SURFACE_BOAT = 11
    MAV_TYPE_SUBMARINE = 12
    MAV_TYPE_HEXAROTOR = 13
    MAV_TYPE_OCTOROTOR = 14
    MAV_TYPE_TRICOPTER = 15
    MAV_TYPE_FLAPPING_WING = 16
    MAV_TYPE_KITE = 17
    MAV_TYPE_ONBOARD_CONTROLLER = 18
    MAV_TYPE_VTOL_DUAL_ROTOR = 19
    MAV_TYPE_VTOL_QUADROTOR = 20
    MAV_TYPE_VTOL_TILTROTOR = 21
    MAV_TYPE_VTOL_RESERVED2 = 22
    MAV_TYPE_VTOL_RESERVED3 = 23
    MAV_TYPE_VTOL_RESERVED4 = 24
    MAV_TYPE_VTOL_RESERVED5 = 25
    MAV_TYPE_GIMBAL = 26
    MAV_TYPE_ADSB = 27
    MAV_TYPE_PARAFOIL = 28
    MAV_TYPE_DODECAROTOR = 29
    MAV_TYPE_CAMERA = 30
    MAV_TYPE_CHARGING_STATION = 31
    MAV_TYPE_FLARM = 32
    MAV_TYPE_SERVO = 33
    MAV_TYPE_ODOMETER = 34
    MAV_TYPE_VICON = 35
    MAV_TYPE_TUNNEL_BEACON = 36
    MAV_TYPE_BATTERY = 37
    MAV_TYPE_FUEL_CELL = 38
    MAV_TYPE_HEAVY_DUTY_WEAPON = 39
    MAV_TYPE_LIGHT_DUTY_WEAPON = 40
    MAV_TYPE_SMALL_DRONE = 41
    MAV_TYPE_MEDIUM_DRONE = 42
    MAV_TYPE_LARGE_DRONE = 43
    MAV_TYPE_NAV_STATION = 44
    MAV_TYPE_RESERVED_45 = 45

class MAV_MODE_FLAG(IntFlag):
    """
    System mode bitmap encoding the MAV's capabilities and state.
    """
    SAFETY_ARMED = 128
    MANUAL_INPUT_ENABLED = 64
    HIL_ENABLED = 32
    STABILIZE_ENABLED = 16
    GUIDED_ENABLED = 8
    AUTO_ENABLED = 4
    TEST_ENABLED = 2
    CUSTOM_MODE_ENABLED = 1

class MAV_STATE(IntEnum):
    """
    MAVLink System State (MAV_STATE)
    States are mutually exclusive.
    """
    MAV_STATE_UNINIT = 0      # Uninitialized system, unknown status.
    MAV_STATE_BOOT = 1        # System is booting up.
    MAV_STATE_CALIBRATING = 2 # System is calibrating and not ready for flight.
    MAV_STATE_STANDBY = 3     # System is grounded and on standby. It can be armed.
    MAV_STATE_ACTIVE = 4      # System is armed and or flying.
    MAV_STATE_CRITICAL = 5    # System is in a critical fail-safe condition.
    MAV_STATE_EMERGENCY = 6   # System is in a recovery mode (e.g. terminating).
    MAV_STATE_POWEROFF = 7    # System is just about to be shutdown.
    MAV_STATE_FLIGHT_TERMINATION = 8 # System is terminated.

class HIL_ACTUATOR_CTL_FLAG(IntFlag):
    """
    Flags specifically for the HIL_ACTUATOR_CONTROLS message.
    """
    LOCKSTEP = 1  # Wait for simulation sync


# MAVLINK Containers #


class MavWrapper(BaseModel):
    def to_list(self) -> list[float]:
        flat_list = []
        
        # Access model_fields from the Class (type(self)) 
        # instead of the instance (self)
        for field_name in type(self).model_fields:
            value = getattr(self, field_name)
            
            if isinstance(value, list):
                flat_list.extend([float(x) for x in value])
            else:
                flat_list.append(float(value))
                
        return flat_list

class HIL_ACTUATOR_CTL(MavWrapper):
    """Actuator commands received from the flight controller."""
    time_usec: int = 0
    controls: list[float] = Field(
        default_factory=lambda: [0.0] * NUM_ACTUATORS,
        min_length=NUM_ACTUATORS, 
        max_length=NUM_ACTUATORS
    )
    mode: MAV_MODE_FLAG = MAV_MODE_FLAG.CUSTOM_MODE_ENABLED
    flags: HIL_ACTUATOR_CTL_FLAG = HIL_ACTUATOR_CTL_FLAG.LOCKSTEP

class HIL_SENSOR(MavWrapper):
    """
    IMU and environmental sensor data sent to the flight controller.
    
    :var time_usec: Timestamp (microseconds since system boot).
    :var xacc, yacc, zacc: Linear acceleration (m/s^2).
    :var xgyro, ygyro, zgyro: Angular velocity (rad/s).
    :var xmag, ymag, zmag: Magnetic field (Gauss).
    :var abs_pressure: Absolute pressure (hPa).
    :var diff_pressure: Differential pressure / Pitot (hPa).
    :var pressure_alt: Altitude from barometer (m).
    :var temp: Temperature (Celsius).
    :var fields_updated: Bitmask of specific sensors updated in this packet.
    """
    time_usec: int = 0
    xacc: float = 0.0
    yacc: float = 0.0
    zacc: float = 0.0
    xgyro: float = 0.0
    ygyro: float = 0.0
    zgyro: float = 0.0
    xmag: float = 0.0
    ymag: float = 0.0
    zmag: float = 0.0
    abs_pressure: float = 0.0
    diff_pressure: float = 0.0
    pressure_alt: float = 0.0
    temp: float = 0.0
    fields_updated: HIL_SENSOR_UPDATE_FLAG = Field(default=HIL_SENSOR_UPDATE_FLAG.RESET)

class HIL_GPS(MavWrapper):
    """
    GPS telemetry sent to the flight controller.
    
    :var time_usec: Timestamp (microseconds).
    :var fix_type: 0-1: no fix, 2: 2D, 3: 3D, 4: DGPS, 5: RTK float, 6: RTK fixed.
    :var lat_degE7: Latitude (WGS84) * 1E7.
    :var lon_degE7: Longitude (WGS84) * 1E7.
    :var alt_mm: Altitude (MSL) in millimeters.
    :var eph: GPS HDOP horizontal dilution of position (cm).
    :var epv: GPS VDOP vertical dilution of position (cm).
    :var vel_cms: GPS ground speed (cm/s).
    :var vn_cms, ve_cms, vd_cms: North, East, Down velocity (cm/s).
    :var cog: Course over ground (centidegrees, 0..35999).
    :var satellites_visible: Number of satellites visible (0-255).
    """
    time_usec: int = 0
    fix_type: GPS_FIX_TYPE = GPS_FIX_TYPE.NO_GPS
    lat_degE7: int = 0
    lon_degE7: int = 0
    alt_mm: int = 0
    eph: int = UINT16_MAX
    epv: int = UINT16_MAX
    vel_cms: int = 0
    vn_cms: int = 0
    ve_cms: int = 0
    vd_cms: int = 0
    cog: int = UINT16_MAX
    satellites_visible: int = UINT8_MAX

class HIL_RC_INPUTS(MavWrapper):
    """
    Simulated Radio Control (RC) inputs sent to the flight controller.
    
    :var chanX_raw: Pulse Width Modulation (PWM) value for channel X (usually 1000-2000).
    :var rssi: Received Signal Strength Indicator (0-255).
    """
    time_usec: int = 0
    chan1_raw: int = 0
    chan2_raw: int = 0
    chan3_raw: int = 0
    chan4_raw: int = 0
    chan5_raw: int = 0 
    chan6_raw: int = 0 
    chan7_raw: int = 0 
    chan8_raw: int = 0 
    chan9_raw: int = 0 
    chan10_raw: int = 0 
    chan11_raw: int = 0 
    chan12_raw: int = 0 
    rssi: int = UINT8_MAX

class HIL_STATE_QUAT(MavWrapper):
    """
    Complete simulated vehicle state in quaternion format.
    
    :var attitude_quaternion: Quaternion orientation [w, x, y, z].
    :var rollspeed_rads: Roll angular speed (rad/s).
    :var lat_degE7: Latitude in degrees * 1E7.
    :var vx_cms: Ground X Speed (cm/s).
    :var ind_airspeed_cms: Indicated Airspeed (cm/s).
    :var xacc_mG: X acceleration in milligravity (mG).
    """
    time_usec: int = 0
    attitude_quaternion: list[float] = Field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])
    rollspeed_rads: float = 0.0
    pitchspeed_rads: float = 0.0
    yawspeed_rads: float = 0.0
    lat_degE7: int = 0
    lon_degE7: int = 0
    alt_mm: int = 0
    vx_cms: int = 0
    vy_cms: int = 0
    vz_cms: int = 0
    ind_airspeed_cms: int = 0
    true_airspeed_cms: int = 0
    xacc_mG: int = 0
    yacc_mG: int = 0
    zacc_mG: int = 0

class HIL_HEARTBEAT(MavWrapper):
    """
    Represents the MAVLink HEARTBEAT message (ID #0).
    The heartbeat message advertises a system's presence, type, and basic state.
    It is sent at a regular rate (usually 1Hz) to maintain the connection.
    """
    type: MAV_TYPE = MAV_TYPE.MAV_TYPE_GENERIC
    autopilot: MAV_AUTOPILOT = MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC
    base_mode: MAV_MODE_FLAG = MAV_MODE_FLAG.CUSTOM_MODE_ENABLED
    custom_mode: int = 0
    system_status: MAV_STATE = MAV_STATE.MAV_STATE_UNINIT
    mavlink_version: int = 3 # default state

class HIL_SYSTEM_TIME(MavWrapper):
    """
    Represents the MAVLink SYSTEM_TIME message (ID #2).
    Used to synchronize the system time between the simulator and the autopilot.
    """
    time_unix_usec: int = 0
    time_boot_ms: int = 0


# Transformation/State Containers #


class State(BaseModel):
    @classmethod
    def get_num_members(cls) -> int:
        return len(cls.model_fields)
    
    def to_list(self) -> list[float]:
        return list(self.model_dump().values())
    
    # TODO, add from_list()

class EF_acc(State):
    """Earth Frame Acceleration components (North-East-Down)."""
    Axe: float = 0.0
    Aye: float = 0.0
    Aze: float = 0.0

class EF_velo(State):
    """Earth Frame Velocity components (North-East-Down)."""
    Vxe: float = 0.0
    Vye: float = 0.0
    Vze: float = 0.0

class EF_pos(State):
    """Earth Frame Position coordinates (North-East-Down)."""
    Xe: float = 0.0
    Ye: float = 0.0
    Ze: float = 0.0

class EULER_ANGLES(State):
    """Vehicle Attitude in Euler Angles (Roll, Pitch, Yaw)."""
    Phi: float = 0.0
    Theta: float = 0.0
    Psi: float = 0.0

class BF_velo(State):
    """Body Frame Velocity (Forward, Right, Down)."""
    U: float = 0.0
    V: float = 0.0
    W: float = 0.0

class BF_ang_rate(State):
    """Body Frame Angular Rates (p, q, r)."""
    p: float = 0.0
    q: float = 0.0
    r: float = 0.0

class BF_acc(State):
    """Body Frame Linear Accelerations."""
    U_dot: float = 0.0
    V_dot: float = 0.0
    W_dot: float = 0.0

class BF_ang_acc(State):
    """Body Frame Angular Accelerations."""
    p_dot: float = 0.0
    q_dot: float = 0.0
    r_dot: float = 0.0   

class STATES(BaseModel):
    """Master state container for vehicle telemetry and dynamics."""
    time_usec: int = Field(default=0, ge=0)
    earth_frame_v: EF_velo = Field(default_factory=EF_velo)
    earth_frame_x: EF_pos = Field(default_factory=EF_pos)
    earth_frame_a: EF_acc = Field(default_factory=EF_acc)
    euler_angles: EULER_ANGLES = Field(default_factory=EULER_ANGLES)
    body_frame_v: BF_velo = Field(default_factory=BF_velo)
    body_frame_w: BF_ang_rate = Field(default_factory=BF_ang_rate)
    body_frame_a: BF_acc = Field(default_factory=BF_acc)
    body_frame_al: BF_ang_acc = Field(default_factory=BF_ang_acc)

    @classmethod
    def get_num_states(cls) -> int:
        total_count = 0
        # model_fields.values() gives us the FieldInfo objects
        for field_info in cls.model_fields.values():
            field_type = field_info.annotation
            if hasattr(field_type, 'get_num_members'):
                total_count += field_type.get_num_members()
        return total_count
    
    # TODO, add from_list()

class DCM(BaseModel):
    """Direction Cosine Matrix for frame transformations."""
    matrix: list[list[float]] = Field(
        default_factory=lambda: [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]
        ]        
    )
    @classmethod
    def get_num_members(cls) -> int:
        return 9 # TODO find number of DCM members
    
    # TODO, add from_list()


# Vehicle Specific Containers


class VEHICLE_COORDINATES(BaseModel):
    lat_degE7: int = 0
    lon_degE7: int = 0

class VEHICLE_ALTITUDE(BaseModel):
    alt_mm: int = 0

class VEHICLE_POSITION(BaseModel):
    coordinates: VEHICLE_COORDINATES = Field(default_factory=VEHICLE_COORDINATES)
    altitude: VEHICLE_ALTITUDE = Field(default_factory=VEHICLE_ALTITUDE)

class VEHICLE_ENVIORNMENT(BaseModel):
    pressure_inHg: float = 0.0
    temp_C: float = 0.0


# Communication Envelopes # 


class HIL_SEND(BaseModel):
    """Outbound data packet sent from simulation bridge to PX4."""
    sensor: HIL_SENSOR = Field(default_factory=HIL_SENSOR)
    gps: HIL_GPS = Field(default_factory=HIL_GPS)
    rc_inputs: HIL_RC_INPUTS = Field(default_factory=HIL_RC_INPUTS)
    quat: HIL_STATE_QUAT = Field(default_factory=HIL_STATE_QUAT)
    heartbeat: HIL_HEARTBEAT = Field(default_factory=HIL_HEARTBEAT)
    time: HIL_SYSTEM_TIME = Field(default_factory=HIL_SYSTEM_TIME)
    flag: HIL_SEND_UPDATE_FLAG = HIL_SEND_UPDATE_FLAG.NONE
    # TODO, add from_list(), maybe not because each member sent at different time...

class HIL_REC(BaseModel):
    """Inbound data packet received from PX4 sent to bridge."""
    actuator_controls: HIL_ACTUATOR_CTL = Field(default_factory=HIL_ACTUATOR_CTL)
    heartbeat: HIL_HEARTBEAT = Field(default_factory=HIL_HEARTBEAT)
    time: HIL_SYSTEM_TIME = Field(default_factory=HIL_SYSTEM_TIME)
    # TODO, add from_list()

class HIL_VEHICLE_STATE(BaseModel):
    """Outbounds data packet sent from vehicle to simulation bridge"""
    states: STATES = Field(default_factory=STATES)
    dcm: DCM = Field(default_factory=DCM)
    pos: VEHICLE_POSITION = Field(default_factory=VEHICLE_POSITION)
    env: VEHICLE_ENVIORNMENT = Field(default_factory=VEHICLE_ENVIORNMENT)