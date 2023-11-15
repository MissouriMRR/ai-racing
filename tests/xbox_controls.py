"""
Loads an ADRL environment, starts a race against a baseline drone, 
and reads input from an Xbox controller to allow user flight control.
"""

import threading
import time
import math
from inputs import get_gamepad
import airsimdroneracinglab

class XboxController(object):
    """
    Class defining commands that enable the reading of Xbox controller inputs.
    """

    MAX_TRIG_VAL = math.pow(2, 8)
    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self):
        self.LeftJoystickY = 0
        self.LeftJoystickX = 0
        self.RightJoystickY = 0
        self.RightJoystickX = 0
        self.LeftTrigger = 0
        self.RightTrigger = 0
        self.LeftBumper = 0
        self.RightBumper = 0
        self.A = 0
        self.X = 0
        self.Y = 0
        self.B = 0
        self.LeftThumb = 0
        self.RightThumb = 0
        self.Back = 0
        self.Start = 0
        self.LeftDPad = 0
        self.RightDPad = 0
        self.UpDPad = 0
        self.DownDPad = 0

        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()


    def read(self):
        """
        Reads various Xbox controller inputs and returns them as a dictionary.

        Returns:
        --------
            dict(str, float): Dictionary of current Xbox controller inputs.
        """
        x_input = self.X
        throttle_input = round(self.LeftJoystickY, 3)
        yaw_rate_input = round(self.LeftJoystickX, 3)
        pitch_input = round(self.RightJoystickY, 3)
        roll_input = round(self.RightJoystickX, 3)
        return {
            "throttle": throttle_input, "yaw_rate": yaw_rate_input, 
            "pitch": pitch_input, "roll": roll_input, "X": x_input
            }


    def _monitor_controller(self):
        while True:
            events = get_gamepad()
            for event in events:
                if event.code == 'ABS_Y':
                    self.LeftJoystickY = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_X':
                    self.LeftJoystickX = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_RY':
                    self.RightJoystickY = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_RX':
                    self.RightJoystickX = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_Z':
                    self.LeftTrigger = event.state / XboxController.MAX_TRIG_VAL # normalize between 0 and 1
                elif event.code == 'ABS_RZ':
                    self.RightTrigger = event.state / XboxController.MAX_TRIG_VAL # normalize between 0 and 1
                elif event.code == 'BTN_TL':
                    self.LeftBumper = event.state
                elif event.code == 'BTN_TR':
                    self.RightBumper = event.state
                elif event.code == 'BTN_SOUTH':
                    self.A = event.state
                elif event.code == 'BTN_NORTH':
                    self.Y = event.state #previously switched with X
                elif event.code == 'BTN_WEST':
                    self.X = event.state #previously switched with Y
                elif event.code == 'BTN_EAST':
                    self.B = event.state
                elif event.code == 'BTN_THUMBL':
                    self.LeftThumb = event.state
                elif event.code == 'BTN_THUMBR':
                    self.RightThumb = event.state
                elif event.code == 'BTN_SELECT':
                    self.Back = event.state
                elif event.code == 'BTN_START':
                    self.Start = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY1':
                    self.LeftDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY2':
                    self.RightDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY3':
                    self.UpDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY4':
                    self.DownDPad = event.state


class ReproduceResetRaceCondition:
    """
    Contains vaious functions for interacting with the ADRL unreal environment
    """
    def __init__(self, drone_name="drone_1"):
        self.airsim_client = airsimdroneracinglab.MultirotorClient()
        self.airsim_client_2 = airsimdroneracinglab.MultirotorClient()
        self.airsim_client_3 = airsimdroneracinglab.MultirotorClient()
        self.drone_name = drone_name
        self.is_thread_active = False
        self.thread_reset = threading.Thread(
            target=self.repeat_timer, args=(self.reset, 0.05)
        )
        self.thread_reset_race = threading.Thread(
            target=self.repeat_timer, args=(self.reset_race, 0.03)
        )
        self.thread_reset_and_reset_race = threading.Thread(
            target=self.repeat_timer, args=(self.reset_and_reset_race, 0.09)
        )
        self.is_thread_active = False

    def repeat_timer(self, callback, period):
        """Simple sleep timer"""
        while self.is_thread_active:
            callback()
            time.sleep(period)

    def load_level(self, level_name, sleep_sec=2.0):
        """
        Loads simulator level.

        Args:
        -----
            level_name (string): Soccer_Field_Easy, Soccer_Field_Medium, ZhangJiaJie_Medium, 
            Building99_Hard, Qualification_Tier_1, Qualification_Tier_2, Qualification_Tier_3, 
            Final_Tier_1, Final_Tier_2, or Final_Tier_3
            sleep_sec (float, optional): Sleep time for loading level. Defaults to 2.0.
        """
        self.level_name = level_name
        self.airsim_client.simLoadLevel(self.level_name)
        self.airsim_client.confirmConnection()  # failsafe
        time.sleep(sleep_sec)  # let the environment load completely

    def reset(self):
        """Resets Airsim cleint."""
        print(time.time(), "called reset")
        self.airsim_client.reset()

    def reset_race(self):
        """Resets current race."""
        print(time.time(), "called simResetRace")
        self.airsim_client_2.simResetRace()

    def reset_and_reset_race(self):
        """Resets Airsim cleint and current race"""
        print(time.time(), "called reset, followed by simResetRace")
        self.airsim_client_3.reset()
        self.airsim_client_3.simResetRace()

    def start_race(self, tier):
        """Starts race against baseline drone"""
        print(time.time(), "called start race")
        self.airsim_client.simStartRace(tier)

    def initialize_drone(self):
        """
        Initializes user drone, enabling API control and arming the vehicle.
        Sets default values for trajectory tracker gains.
        
        """
        self.airsim_client.enableApiControl(vehicle_name=self.drone_name)
        self.airsim_client.arm(vehicle_name=self.drone_name)

        # set default values for trajectory tracker gains
        traj_tracker_gains = airsimdroneracinglab.TrajectoryTrackerGains(
            kp_cross_track=5.0,
            kd_cross_track=0.0,
            kp_vel_cross_track=3.0,
            kd_vel_cross_track=0.0,
            kp_along_track=0.4,
            kd_along_track=0.0,
            kp_vel_along_track=0.04,
            kd_vel_along_track=0.0,
            kp_z_track=2.0,
            kd_z_track=0.0,
            kp_vel_z=0.4,
            kd_vel_z=0.0,
            kp_yaw=3.0,
            kd_yaw=0.1,
        )

        self.airsim_client.setTrajectoryTrackerGains(
            traj_tracker_gains, vehicle_name=self.drone_name
        )
        time.sleep(0.2)

    def start_threads(self):
        """Starts threads if not already active."""
        if not self.is_thread_active:
            self.is_thread_active = True
            self.thread_reset.start()
            self.thread_reset_race.start()
            self.thread_reset_and_reset_race.start()
            print("Started threads")

    def stop_threads(self):
        """Stops threads if not already stopped"""
        if self.is_thread_active:
            self.is_thread_active = False
            self.thread_reset.join()
            self.thread_reset_race.join()
            self.thread_reset_and_reset_race.join()
            print("Stopped threads.")

    def takeoff(self):
        """Built-in ADRL async takeoff function"""
        self.airsim_client.takeoffAsync().join()

    def give_control_stick_inputs(self, roll, pitch, yaw_rate, throttle, duration):
        """
        Reads various control stick inputs passes them to the drone for a given amount of time.

        Args:
            roll (float): an angle in radians
            pitch (float): an angle in radians
            yaw_rate (float): an angle in radians
            throttle (float): a number between 0.0 and 1.0
            duration (float): in seconds

        Notes (from airsimdroneracinglab documenation):
        -----------------------------------------------
            Roll angle, pitch angle, and yaw angle are given in radians, in the body frame.
            The body frame follows the Front Left Up (FLU) convention, and right-handedness.
        """
        self.airsim_client.moveByRollPitchYawrateThrottleAsync(
            roll, pitch, yaw_rate, throttle,
            duration, vehicle_name=self.drone_name
            )
        print('Control stick input passed to drone')
        time.sleep(duration)

if __name__ == "__main__":
    input_interval: float = .2
    controller_deadzone: float = .1
    reproducer = ReproduceResetRaceCondition("drone_1")
    reproducer.load_level("Soccer_Field_Easy")  #Loads Given Level
    reproducer.initialize_drone()
    reproducer.start_race(3)
    joy = XboxController()
    reproducer.takeoff()

# Takes controller input until the "x" button is pressed
    while True:
        inputs = joy.read()
        if inputs["X"] == 1:
            print('Program Quitting')
            break

        # Checks if inputs are below controller deadzone setting
        for key in (inputs["roll"], inputs['pitch'], inputs['yaw_rate'], inputs['throttle']):
            if inputs[key] > controller_deadzone:
                break
        else:
            print("Drone hovering")
            time.sleep(.1)
            continue

        # Sends input commands to drone
        # Note: yaw_rate is counter-intuivive, throttle adjusted for xbox controller stick
        reproducer.give_control_stick_inputs(
            inputs["roll"], inputs['pitch'], -inputs['yaw_rate'],
            (inputs['throttle'] * .4) + .6, input_interval
            )
        