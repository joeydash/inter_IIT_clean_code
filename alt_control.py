from __future__ import print_function
import math

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil  # Needed for command message definitions
import time

from xy_control import xy_position_control


def get_alt(prev_val_read, delta_t, curr_val_read, curr_alt):
    curr_val_read = vehicle.location.global_relative_frame.alt
    if curr_val_read == prev_val_read:
        curr_alt = curr_alt + vehicle.velocity[2] * (delta_t)
    # correction: if feddback is recieved
    else:
        curr_alt = vehicle.location.global_relative_frame.alt
        prev_val_read = curr_alt
    return [curr_alt, prev_val_read]


# -----------------------------------------------------------------------#
# -----------call this function to control altitude----------------------#
# -----------------------------------------------------------------------#

def altitude_control(target_altitude, kp_alt):
    # declaring variables
    prev_val_read = vehicle.location.global_relative_frame.alt
    prev_time = time.time()
    then = time.time()

    curr_alt = vehicle.location.global_relative_frame.alt
    curr_val_read = curr_alt
    while True:

        curr_time = time.time()

        # prediction: updating curr_alt with velocity*dt if alt feedback is not updated
        alt_data = get_alt(prev_val_read, curr_time - prev_time, curr_val_read, curr_alt)
        curr_alt = alt_data[0]
        prev_val_read = alt_data[1]

        # soring values for next loop
        prev_time = curr_time

        # calculating error
        error = target_altitude - curr_alt  # in meters
        # correction
        vel_corr = kp_alt * error  # in meters

        # constraining velocity within -1 to 1
        if vel_corr > 1:
            vel_corr = 1
        elif vel_corr < -1:
            vel_corr = -1

        # mapping velocity to throttle
        if vel_corr > 0:
            thr = 1590 + vel_corr * 400
        elif vel_corr < 0:
            thr = 1410 + vel_corr * 400
        else:
            thr = 1500

        print(thr)
        print(curr_alt)
        vehicle.channels.overrides['3'] = thr

        now = time.time()

        # break loop if it holds altitude for more than 5 secs
        if target_altitude + 0.2 > vehicle.location.global_relative_frame.alt > target_altitude - 0.2:
            if now - then > 5:
                break
        else:
            then = now


def control_main(height_queue, points_to_displace_queue):
    # print "works!"
    i = 1
    while True:
        height_queue.put(i)
        i += 1
    kp_alt = .5
    kp_velx = .5
    kp_vely = .5

    connection_string = '/dev/ttyACM0'
    # connection_string = "tcp:127.0.0.1:5763"
    # Connect to the Vehicle..
    print('Connecting to vehicle on: %s' % connection_string)
    vehicle = connect(connection_string, wait_ready=True)

    print("Basic pre-arm checks")
    vehicle.mode = VehicleMode("LOITER")

    print("Arming motors")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    altitude_control(3, kp_alt)

    vehicle.channels.overrides['3'] = 1500
    print(vehicle.location.global_relative_frame.alt)
    print("holding")
    time.sleep(5)

    # initialising variables
    prev_val_read = vehicle.location.global_relative_frame.alt
    prev_time = time.time()
    then = time.time()

    while True:
        curr_time = time.time()
        alt_data = get_alt(prev_val_read, curr_time - prev_time)
        prev_val_read = alt_data[1]
        height_queue.put(alt_data[0])
        prev_time = curr_time
        while points_to_displace_queue.empty:
            pass
        error_xy = points_to_displace_queue.get()

        msg = xy_position_control(error_xy[1], error_xy[0], kp_velx, kp_vely, vehicle.altitude.yaw)
        vehicle.send_mavlink(msg)
        now = time.time()
        radius = math.sqrt(error_xy[0] ** 2 + error_xy[1] ** 2)
        # break loop if it holds altitude for more than 5 secs
        if radius + 5 > radius > radius - 5:
            if now - then > 5:
                break
        else:
            then = now

    vehicle.mode = VehicleMode("LAND")
    vehicle.close()

    print("Completed")
