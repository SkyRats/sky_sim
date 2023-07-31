import argparse
from dronekit import connect


parser = argparse.ArgumentParser()
parser.add_argument('--connect', default = '127.0.0.1:14550')
args = parser.parse_args()

#-- Connect to the vehicle
print('Connecting...')
vehicle = connect(args.connect)

#-- Check vehicle status
print(f"Mode: {vehicle.mode.name}")
print(" Global Location: %s" % vehicle.location.global_frame)
print(" Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
print(" Local Location: %s" % vehicle.location.local_frame)
print(" Attitude: %s" % vehicle.attitude)
print(" Velocity: %s" % vehicle.velocity)
print(" Gimbal status: %s" % vehicle.gimbal)
print(" EKF OK?: %s" % vehicle.ekf_ok)
print(" Last Heartbeat: %s" % vehicle.last_heartbeat)
print(" Rangefinder: %s" % vehicle.rangefinder)
print(" Rangefinder distance: %s" % vehicle.rangefinder.distance)
print(" Rangefinder voltage: %s" % vehicle.rangefinder.voltage)
print(" Is Armable?: %s" % vehicle.is_armable)
print(" System status: %s" % vehicle.system_status.state)
print(" Armed: %s" % vehicle.armed)    # settable