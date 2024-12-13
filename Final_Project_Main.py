import sys
import TurboPi.HiwonderSDK.mecanum as mecanum
import signal
import time
import numpy as np
import a_star_planner
from threading import Timer

position = np.array([0, 0])
goal_position = None
velocity = np.array([0, 0, 0])  # Linear velocity, Linear direction, Angular velocity

chassis = mecanum.MecanumChassis()

inches_to_mm = 25.4

def stop(signum, frame):
    print("stopping from signal interrupt")
    global goal_position
    global velocity
    goal_position = None
    velocity = ([0, 0, 0])
    chassis.set_velocity(velocity[0], velocity[1], velocity[2])

signal.signal(signal.SIGINT, stop)

# Credit to right2clicky at https://stackoverflow.com/questions/12435211/threading-timer-repeat-function-every-n-seconds
# for the RepeatTimer class
class RepeatTimer(Timer):
    def run(self):
        while not self.finished.wait(self.interval):
            self.function(*self.args, **self.kwargs)

class RobotController:
    '''
    Robot Controller class is responsible for driving the robot from its current position to a goal position
    Also provides some utility functions for other movement
    No other classes/functions should provide robot inputs (except the signal stop function)
    '''

    def __init__(self, hz):
        # hz = speed at which the position and velocity threads should run
        self.update_position_thread = RepeatTimer(hz, self.update_position)
        self.set_velocity_thread = RepeatTimer(hz, self.set_velocity)

    def update_position(self):
        # Run at x Hz updating position based on velocity
        print("Updating Position")

    def set_velocity(self):
        # Run at x Hz setting velocity to drive towards a goal (straight line)
        print("Setting velocity")

    def stop(self):
        # Stops the vehicle
        global velocity
        velocity = ([0, 0, 0])
        chassis.set_velocity(velocity[0], velocity[1], velocity[2])
        print("Vehicle stopped")
    
    def happy_dance(self):
        # Happy (spin, move servos, flash LED)
        print("Happy dancing")

class RobotPlanner:
    # RobotPlanner class plans a route to a goal using A*, and directs the robot to move along the route

    def __init__(self, map: np.ndarray):
        '''
        map: 2D numpy array containing the map of the area. 0 represents open space, 1 represents occupied.
        '''
        self.map = map

    def navigate_to_goal(self, goal: np.ndarray):
        # goal: 1D numpy array containing the coordinates of the goal square
        print(f"Navigating to {goal}")


if __name__ == '__main__':
    map = np.array([[0,1,0,0,0],
                    [0,1,0,0,0],
                    [0,1,0,0,0],
                    [0,1,0,1,0],
                    [0,0,0,1,0]])
    goal = np.array([4, 4])
    controller = RobotController()
    controller.update_position_thread.start()
    controller.set_velocity_thread.start()

    controller.update_position_thread.cancel()
    controller.set_velocity_thread.cancel()