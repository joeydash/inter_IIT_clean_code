
from __future__ import print_function
import multiprocessing
from joeydash import set_point_to_displace
from alt_control import control_main, altitude_control
import math

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil  # Needed for command message definitions
import time

from xy_control import xy_position_control


connection_string = '/dev/ttyACM0'
connection_string = "tcp:127.0.0.1:5763"
# Connect to the Vehicle..
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)
print("works")
print("Basic pre-arm checks")
vehicle.mode = VehicleMode("LOITER")
time.sleep(5)
print(vehicle.mode.name)
print("Arming motors")
vehicle.armed = True
time.sleep(5)
while not vehicle.armed:
    print("Waiting for arming...")
    time.sleep(1)

kp_alt = .5

print("Taking off!")
altitude_control(3, kp_alt,vehicle)

vehicle.channels.overrides['3'] = 1500
print(vehicle.location.global_relative_frame.alt)
print("holding")
time.sleep(5)


def main():
    height_queue = multiprocessing.Queue(1)
    points_to_displace_queue = multiprocessing.Queue(1)
    set_point_to_displace_process = multiprocessing.Process(target=set_point_to_displace, args=(height_queue, points_to_displace_queue))
    control_main_processing = multiprocessing.Process(target=control_main, args=(height_queue, points_to_displace_queue,vehicle))
    set_point_to_displace_process.start()
    control_main_processing.start()
    set_point_to_displace_process.join()
    control_main_processing.join()


if __name__ == "__main__":
    main()
