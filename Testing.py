import sys
import TurboPi.HiwonderSDK.mecanum as mecanum
import signal
import time

chassis = mecanum.MecanumChassis()

def stop(signum, frame):
    print("stopping from signal interrupt")
    chassis.set_velocity(0,0,0)

signal.signal(signal.SIGINT, stop)

if __name__ == '__main__':
    print("driving forward")
    chassis.set_velocity(50, 90, 0)
    time.sleep(10)

    chassis.set_velocity(0, 0, 0)
    print("stopping")