import math
from pymavlink import mavutil


def limit_vel(vel):
    if vel > 0.5:
        vel = 0.5
    elif vel < -0.5:
        vel = -0.5
    return vel


def xy_position_control(error_x, error_y, kp_velx, kp_vely, curr_yaw, vehicle):
    vel_corrx = kp_velx * error_x
    vel_corry = kp_vely * error_y

    vel_coorx = limit_vel(vel_corrx)
    vel_corry = limit_vel(vel_corry)

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        vel_corrx * math.cos(curr_yaw) - vel_corry * math.sin(curr_yaw),
        vel_corrx * math.sin(curr_yaw) + vel_corry * math.cos(curr_yaw), 0,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    print("The north speed is ", vel_coorx * math.cos(curr_yaw) - vel_coorx * math.sin(curr_yaw))
    print("The EAST speed is ", vel_corry * math.sin(curr_yaw) + vel_corry * math.cos(curr_yaw))

    return msg
