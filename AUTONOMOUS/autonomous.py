import time

import math
from dronekit import VehicleMode

#from mavlink import sonar_alt

DEFAULT_TAKEOFF_THRUST = 0.7
SMOOTH_TAKEOFF_THRUST = 0.6
class Autonomous(object):
    def __init__(self,vehicle):
        self.vehicle = vehicle

    def arm_and_takeoff_nogps(self,aTargetAltitude):
        print("Basic pre-arm checks")
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)
        print("Arming motors")
        # Copter should arm in GUIDED_NOGPS mode
        self.vehicle.mode = VehicleMode("GUIDED_NOGPS")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            print(" Waiting for arming...")
            self.vehicle.armed = True
            time.sleep(1)
        print("Taking off!")

        thrust = DEFAULT_TAKEOFF_THRUST
        while True:
            current_altitude = self.vehicle.location.global_relative_frame.alt
            #current_altitude = sonar_alt
            print(" Altitude: %f  Desired: %f" %
                  (current_altitude, aTargetAltitude))
            if current_altitude >= aTargetAltitude * 0.95:  # Trigger just below target alt.
                print("Reached target altitude")
                break
            elif current_altitude >= aTargetAltitude * 0.6:
                thrust = SMOOTH_TAKEOFF_THRUST
            self.set_attitude(thrust=thrust)
            time.sleep(0.2)

    def set_attitude(self,roll_angle=0.0, pitch_angle=0.0, yaw_rate=0.0, thrust=0.5, duration=0):
        # Thrust >  0.5: Ascend
        # Thrust == 0.5: Hold the altitude
        # Thrust <  0.5: Descend
        msg = self.vehicle.message_factory.set_attitude_target_encode(
            0,  # time_boot_ms
            1,  # Target system
            1,  # Target component
            0b00000000,  # Type mask: bit 1 is LSB
            self.to_quaternion(roll_angle, pitch_angle),  # Quaternion
            0,  # Body roll rate in radian
            0,  # Body pitch rate in radian
            math.radians(yaw_rate),  # Body yaw rate in radian
            thrust  # Thrust
        )
        self.vehicle.send_mavlink(msg)

        start = time.time()
        while time.time() - start < duration:
            self.vehicle.send_mavlink(msg)
            time.sleep(0.1)

    def to_quaternion(self,roll=0.0, pitch=0.0, yaw=0.0):
        """
        Convert degrees to quaternions
        """
        t0 = math.cos(math.radians(yaw * 0.5))
        t1 = math.sin(math.radians(yaw * 0.5))
        t2 = math.cos(math.radians(roll * 0.5))
        t3 = math.sin(math.radians(roll * 0.5))
        t4 = math.cos(math.radians(pitch * 0.5))
        t5 = math.sin(math.radians(pitch * 0.5))

        w = t0 * t2 * t4 + t1 * t3 * t5
        x = t0 * t3 * t4 - t1 * t2 * t5
        y = t0 * t2 * t5 + t1 * t3 * t4
        z = t1 * t2 * t4 - t0 * t3 * t5

        return [w, x, y, z]