import sys
import TurboPi.HiwonderSDK.mecanum as mecanum
import signal
import time
import numpy as np
import a_star_planner
from threading import Timer

position = (0, 0)   # Position is in units of 1' x 1' grid squares
goal_position = None
velocity = (0, 0, 0)  # Linear velocity (in/s), Linear direction (rad), Angular velocity (deg / half s)

chassis = mecanum.MecanumChassis()

inches_to_mm = 25.4
ft_per_sec = 12 * inches_to_mm
deg_to_rad = 180 / np.pi

def stop(signum, frame):
    print("stopping from signal interrupt")
    global goal_position
    global velocity
    goal_position = None
    velocity = (0, 0, 0)
    chassis.set_velocity(velocity[0], velocity[1] * deg_to_rad, velocity[2])

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
        self.set_velocity_to_goal_thread = RepeatTimer(hz, self.set_velocity_to_goal)

    # Update robot position
    def update_position(self):
        global position

        position[0] = position[0] + velocity[0] * np.cos(velocity[1])
        position[1] = position[1] + velocity[0] * np.sin(velocity[1])

    # Drive the robot to the current goal
    def set_velocity_to_goal(self):
        global velocity

        if (goal_position is not None):
            theta = np.atan2(position[1] - goal_position[1], position[0] - goal_position[0])
            velocity = (ft_per_sec, theta, 0)
        else:
            velocity = (0, 0, 0)
        chassis.set_velocity(velocity[0], velocity[1] * deg_to_rad, velocity[2])

    def stop(self):
        # Stops the vehicle
        global velocity
        velocity = (0, 0, 0)
        chassis.set_velocity(velocity[0], velocity[1] * deg_to_rad, velocity[2])
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


    def navigate_to_final_goal(self, final_goal: np.ndarray):
        # goal: 1D numpy array containing the coordinates of the goal square
        self.plan = a_star_planner.a_star_grid(self.map, (np.floor(position[0]).item(), np.floor(position[1]).item()), final_goal)

        goal_position = self.plan.pop().name

        while (True):
            if (self.close_to_goal()):
                if goal_position == final_goal:
                    break
                print(f"reached point {goal_position}")
                goal_position = self.plan.pop().name
            else:
                time.sleep(0.1)

        print(f"Reached {final_goal}")

    # Returns true if position is less than 0.1 units from the current goal
    def close_to_goal(self):
        if np.sqrt((position[0] - goal_position[0]) ** 2 + (position[1] - goal_position[1]) ** 2) < 0.1:
            return True
        else: 
            return False


if __name__ == '__main__':
    map = np.array([[0,1,0,0,0],
                    [0,1,0,0,0],
                    [0,1,0,0,0],
                    [0,1,0,1,0],
                    [0,0,0,1,0]])
    final_goal = (4, 4)
    controller = RobotController(30)
    controller.update_position_thread.start()
    controller.set_velocity_to_goal_thread.start()

    planner = RobotPlanner(map)
    planner.navigate_to_final_goal(final_goal)

    controller.update_position_thread.cancel()
    controller.set_velocity_to_goal_thread.cancel()