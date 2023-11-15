"""
Loads an ADRL environment, starts a race against a baseline drone, 
and reads input from terminal to control user drone.
"""

import threading
import time
import math
import airsimdroneracinglab



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

    def give_control_stick_inputs(self, roll, pitch, yaw, throttle, duration):
        """
        Reads various control stick inputs and passes them to the drone for a given amount of time.

        Args:
        -----
            roll (float): angle in radians
            pitch (float): angle in radians
            yaw (float): angle in radians
            throttle (float): a number from 0.0 to 1.0
            duration (float): in seconds

        Notes (from airsimdroneracinglab documenation):
        -----------------------------------------------
            Roll angle, pitch angle, and yaw angle are given in radians, in the body frame.
            The body frame follows the Front Left Up (FLU) convention, and right-handedness.
        """
        self.airsim_client.moveByRollPitchYawThrottleAsync(
            roll, pitch, yaw, throttle, duration,
            vehicle_name=self.drone_name
            )
        print('Control stick input passed to drone')

def get_user_inputs():
    """
    Prompts the terminal for drone control inputs in degrees, then converts them to radians.

    Returns:
    --------
        dict(str, float): Dictionary of inputs for drone control
    """
    roll_input = math.radians(float(input('Roll (in degrees):')))
    pitch_input = math.radians(float(input('Pitch (in degrees):')))
    yaw_input = math.radians(float(input('Yaw (in radians):')))
    throttle_input = float(input('Throttle (0.0-1.0):'))
    duration_input = float(input("Input Duration (seconds):"))
    return {
        "roll": roll_input, "pitch": pitch_input,
        "yaw": yaw_input, "throttle": throttle_input, "duration": duration_input
        }

if __name__ == "__main__":
    reproducer = ReproduceResetRaceCondition("drone_1")
    reproducer.load_level("Soccer_Field_Easy")
    reproducer.initialize_drone()
    reproducer.start_race(3)
    reproducer.takeoff()


    while True:
        inputs = get_user_inputs()
        reproducer.give_control_stick_inputs(
            inputs["roll"], inputs['pitch'],
            inputs['yaw'], inputs['throttle'], inputs['duration']
            )

        if input('Continue? (yes or no):') == 'no':
            print('Shutting down.')
            break
