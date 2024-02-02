"""
Based on the test_reset.py file for much of the environment setup.
Loads a level, initiallizes a drone, takes off, then prompts the user 
for manual-mode inputs and passes these inputs to the drone.
"""
import time
import math
import threading
import airsimdroneracinglab


class ReproduceResetRaceCondition:
    """
    Contains various functions for interacting with the ADRL unreal environment.

    Attributes
    ----------
    airsim_client : airsimdroneracinglab.client.MultirotorCleint
        Default airsim client for interacting with the environment
    airsim_client_2 : airsimdroneracinglab.client.MultirotorCleint
        Second alternative airsim client for interacting with the environment
    airsim_client_3 : airsimdroneracinglab.client.MultirotorCleint
        Third alternative airsim client for interacting with the environment
    drone_name : string
        Name of user controlled drone.
    is_thread_active : boolean
        Boolean used to see if current thread is active or not
    thread_reset : threading.Thread
        Thread used to reset airsim cleint.
    thread_reset_race : threading.Thread
        Thread used to reset current race
    thread_reset_and_reset_race : threading.Thread
        Thread used to reset airsim client and current race
    level_name : string
        Name of current loaded level (see load_level() for options)

    Methods
    -------
    repeat_timer(callback, period: float)
        Simple sleep timer
    load_level(level_name: str, sleep_sec: float = 2.0)
        Loads given simulator level
    reset()
        Resets airsim client
    reset_race()
        Resets current race
    reset_and_reset_race()
        Resets airsim cleint and current race
    start_race(tier: int = 1)
        Starts race against a baseline drone using moveonspline
    initialize_drone()
        Initializes user drone, enabling API control and arming the vehicle.
        Sets default values for trajectory tracker gains.
    start_threads()
        Starts threads if not already active.
    stop_threads()
        Stops threads if not already stopped
    takeoff()
        Built-in ADRL async takeoff function. Sends moveonspline commands to drone in order to take off.
    give_control_stick_inputs(self, roll, pitch, yaw, throttle, duration)
        Reads various control stick inputs from command-line interface
        and passes them to the drone for a given amount of time.

    """

    def __init__(self, drone_name: str = "drone_1") -> None:
        """
        Initializes class variables.

        Parameters
        ----------
            drone_name : string
                Name of user controlled drone (defaults to 'drone_1')
        """
        self.airsim_client: airsimdroneracinglab.client.MultirotorClient = (
            airsimdroneracinglab.MultirotorClient()
        )
        self.airsim_client_2: airsimdroneracinglab.client.MultirotorClient = (
            airsimdroneracinglab.MultirotorClient()
        )
        self.airsim_client_3: airsimdroneracinglab.client.MultirotorClient = (
            airsimdroneracinglab.MultirotorClient()
        )
        self.drone_name: str = drone_name
        self.is_thread_active: bool = False
        self.thread_reset: threading.Thread = threading.Thread(
            target=self.repeat_timer, args=(self.reset, 0.05)
        )
        self.thread_reset_race: threading.Thread = threading.Thread(
            target=self.repeat_timer, args=(self.reset_race, 0.03)
        )
        self.thread_reset_and_reset_race: threading.Thread = threading.Thread(
            target=self.repeat_timer, args=(self.reset_and_reset_race, 0.09)
        )
        self.is_thread_active = False

    def repeat_timer(self, callback, period: float) -> None:
        """
        Simple sleep timer.

        Parameters
        ----------
            callback
                Function to call
            period : float
                Repeat interval in seconds
        """
        while self.is_thread_active:
            callback()
            time.sleep(period)

    def load_level(self, level_name: str, sleep_sec: float = 2.0) -> None:
        """
        Loads given simulator level.

        Parameters
        ----------
            level_name : string
                Soccer_Field_Easy, Soccer_Field_Medium, ZhangJiaJie_Medium,
                Building99_Hard, Qualification_Tier_1, Qualification_Tier_2,
                Qualification_Tier_3, Final_Tier_1, Final_Tier_2, or Final_Tier_3
            sleep_sec : float, default=2.0
                Sleep time for loading level.
        """
        self.level_name: str = level_name
        self.airsim_client.simLoadLevel(self.level_name)
        self.airsim_client.confirmConnection()  # failsafe
        time.sleep(sleep_sec)  # let the environment load completely

    def reset(self) -> None:
        """Resets Airsim cleint."""
        print(time.time(), "called reset")
        self.airsim_client.reset()

    def reset_race(self) -> None:
        """Resets current race."""
        print(time.time(), "called simResetRace")
        self.airsim_client_2.simResetRace()

    def reset_and_reset_race(self) -> None:
        """Resets airsim cleint and current race"""
        print(time.time(), "called reset, followed by simResetRace")
        self.airsim_client_3.reset()
        self.airsim_client_3.simResetRace()

    def start_race(self, tier: int = 1) -> None:
        """
        Starts race against a baseline drone using moveonspline

        Parameters
        ----------
            tier : int, default=1
                Race tier determining difficulty.
                Tier 1: Planning only; Tier 2: Perception Only; Tier 3: Planning and Perception.
        """
        print(time.time(), "called start race")
        self.airsim_client.simStartRace(tier)

    def initialize_drone(self) -> None:
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

    def start_threads(self) -> None:
        """Starts threads if not already active."""
        if not self.is_thread_active:
            self.is_thread_active = True
            self.thread_reset.start()
            self.thread_reset_race.start()
            self.thread_reset_and_reset_race.start()
            print("Started threads")

    def stop_threads(self) -> None:
        """Stops threads if not already stopped."""
        if self.is_thread_active:
            self.is_thread_active = False
            self.thread_reset.join()
            self.thread_reset_race.join()
            self.thread_reset_and_reset_race.join()
            print("Stopped threads.")

    def takeoff(self) -> None:
        """Built-in ADRL async takeoff function. Sends moveonspline commands to drone in order to take off."""
        self.airsim_client.takeoffAsync().join()

    def give_control_stick_inputs(
        self, roll: float, pitch: float, yaw: float, throttle: float, duration: float
    ) -> None:
        """
        Reads various control stick inputs from command-line interface
        and passes them to the drone for a given amount of time.
        Drone automatically attempts to stabilize itself once commands are finished.

        Parameters
        ----------
            roll : float
                Roll angle to be passed to drone given in radians
            pitch : float
                Pitch angle to be passed to drone given in radians
            yaw : float
                Yaw angle to be passed to drone given in radians
            throttle : float
                A throttle value from 0.0 to 1.0 (Neutral/Hover is about 0.5938 for the default drone)
            duration : float
                Duration for the inputs to be passed to the drone given in seconds

        Notes
        -----
            (from airsimdroneracinglab documenation)
            Roll angle, pitch angle, and yaw angle are given in radians, in the body frame.
            The body frame follows the Front Left Up (FLU) convention, and right-handedness.
        """
        self.airsim_client.moveByRollPitchYawThrottleAsync(
            roll, pitch, yaw, throttle, duration, vehicle_name=self.drone_name
        )
        print("Control stick input passed to drone!")


def get_user_inputs() -> dict[str, float]:
    """
    Prompts the terminal for inputs to be passed to the drone.
    Promts the user in degrees for increased usability, then converts them to radians.

    Returns
    -------
        inputs : dict[str, float]
            Dictionary of various inputs for drone control
    """
    roll_input: float = math.radians(float(input("Roll (in degrees):\n")))
    pitch_input: float = math.radians(float(input("Pitch (in degrees):\n")))
    yaw_input: float = math.radians(float(input("Yaw (in degrees):\n")))
    throttle_input: float = float(input("Throttle (0.0-1.0):\n"))
    duration_input: float = float(input("Input Duration (seconds):\n"))
    return {
        "roll": roll_input,
        "pitch": pitch_input,
        "yaw": yaw_input,
        "throttle": throttle_input,
        "duration": duration_input,
    }


if __name__ == "__main__":
    reproducer: ReproduceResetRaceCondition = ReproduceResetRaceCondition("drone_1")
    reproducer.load_level("Soccer_Field_Easy")
    reproducer.initialize_drone()
    reproducer.start_race(3)
    reproducer.takeoff()

    # Loops until the user is done giving input to the drone
    while True:
        inputs: dict[str, float] = get_user_inputs()
        reproducer.give_control_stick_inputs(
            inputs["roll"],
            inputs["pitch"],
            inputs["yaw"],
            inputs["throttle"],
            inputs["duration"],
        )
        time.sleep(
            inputs["duration"]
        )  # Sleeps here until done passing inputs to make the command line UI more intuitive

        if input("Stop? (type 'yes' to stop)\n") == "yes":
            print("Shutting down...")
            break
        print()  # Blank line to help separate lines in console
