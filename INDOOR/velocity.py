import time
from pymavlink import mavutil


class Velocity(object):
    def __init__(self,vehicle,x,y,z,duration):
        self.vehicle=vehicle
        self.x=x
        self.y=y
        self.z=z
        self.duration = duration

    def send_global_velocity(self):
        """
        Move vehicle in direction based on specified velocity vectors.
        """
        msg = self.vehicle.message_factory.set_position_target_global_int_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0,  # lat_int - X Position in WGS84 frame in 1e7 * meters
            0,  # lon_int - Y Position in WGS84 frame in 1e7 * meters
            0,  # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
            # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
            self.x,  # X velocity in NED frame in m/s
            self.y,  # Y velocity in NED frame in m/s
            self.z,  # Z velocity in NED frame in m/s
            0, 0, 0,  # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        # send command to vehicle on 1 Hz cycle
        for x in range(0, self.duration):
            self.vehicle.send_mavlink(msg)
            time.sleep(1)