import sys
import TurboPi.HiwonderSDK.mecanum as mecanum
import signal
import time

inches_to_mm = 25.4

chassis = mecanum.MecanumChassis()

def stop(signum, frame):
    print("stopping from signal interrupt")
    chassis.set_velocity(0,0,0)

signal.signal(signal.SIGINT, stop)

def move_fwd(inches):
    mm = inches * inches_to_mm
    total_drive_time = inches / 12
    current_drive_time = 0
    while current_drive_time < total_drive_time:
        chassis.set_velocity(12*inches_to_mm, 90, 0)
        time.sleep(1)
        current_drive_time += 1
    chassis.set_velocity(0,0,0)

def rotate():
    chassis.set_velocity(0, 0, 45)
    time.sleep(1)
    chassis.set_velocity(0,0,0)   
    time.sleep(1)
    chassis.set_velocity(0, 0, -45)
    time.sleep(1)
    chassis.set_velocity(0,0,0)   

if __name__ == '__main__':
    print("driving forward")
    move_fwd(12)
    time.sleep(2)
    print("Rotating")
    rotate()