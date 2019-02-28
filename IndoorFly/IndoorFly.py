import time
import math
from dronekit import VehicleMode

vehicle=None
class IndoorFlyClass:

    @staticmethod
    def set_vehicle(vehicle_obj):
        global vehicle
        vehicle = vehicle_obj
    @staticmethod
    def fly_indoor_target_alt(alt):
        IndoorFlyClass.arm_and_takeoff_nogps(alt)
    @staticmethod
    def arm_and_takeoff_nogps(aTargetAltitude):
        DEFAULT_TAKEOFF_THRUST = 0.7
        SMOOTH_TAKEOFF_THRUST = 0.6
        vehicle.mode = VehicleMode("GUIDED_NOGPS")
        #vehicle.mode = VehicleMode("GUIDED")
        vehicle.armed = True
        while not vehicle.armed:
            print(" Waiting for arming...")
            vehicle.armed = True
            time.sleep(1)
        print("Taking off!")
        thrust = DEFAULT_TAKEOFF_THRUST
        while True:
            current_altitude = vehicle.location.global_relative_frame.alt
            print(" Altitude: %f  Desired: %f" %(current_altitude, aTargetAltitude))
            if current_altitude >= aTargetAltitude * 0.95:  # Trigger just below target alt.
                print("Reached target altitude")
                break
            elif current_altitude >= aTargetAltitude * 0.6:
                thrust = SMOOTH_TAKEOFF_THRUST
            IndoorFlyClass.set_attitude(thrust=thrust)
            time.sleep(0.2)
    @staticmethod
    def set_attitude(roll_angle=0.0, pitch_angle=0.0, yaw_rate=0.0, thrust=0.5, duration=0):
        msg = vehicle.message_factory.set_attitude_target_encode(
            0,  # time_boot_ms
            1,  # Target system
            1,  # Target component
            0b00000000,  # Type mask: bit 1 is LSB
            IndoorFlyClass.to_quaternion(roll_angle, pitch_angle),  # Quaternion
            0,  # Body roll rate in radian
            0,  # Body pitch rate in radian
            math.radians(yaw_rate),  # Body yaw rate in radian
            thrust  # Thrust
        )
        vehicle.send_mavlink(msg)
        start = time.time()
        while time.time() - start < duration:
            vehicle.send_mavlink(msg)
            time.sleep(0.1)
    @staticmethod
    def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
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
    @staticmethod
    def set_a():
        DEFAULT_TAKEOFF_THRUST = 0.7
        SMOOTH_TAKEOFF_THRUST = 0.6
        vehicle.mode = VehicleMode("GUIDED_NOGPS")
        # vehicle.mode = VehicleMode("GUIDED")
        vehicle.armed = True
        while not vehicle.armed:
            print(" Waiting for arming...")
            vehicle.armed = True
            time.sleep(1)
        thrust = DEFAULT_TAKEOFF_THRUST
        print("Move forward")
        IndoorFlyClass.set_attitude(pitch_angle=-5, thrust=1, duration=3.21)

        print("Move backward")
        IndoorFlyClass.set_attitude(pitch_angle=5, thrust=1, duration=3)