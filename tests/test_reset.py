<<<<<<< HEAD
<<<<<<< HEAD
"""
This file was adapted from a setup file given in 
the https://github.com/microsoft/AirSim-Drone-Racing-Lab repositiry
It simply loads a level and starts a race against a baseline drone
to validate that everything has been set up properly.
"""

<<<<<<< HEAD
import time
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
=======
=======
"""
This file was adapted from a setup file given in 
the https://github.com/microsoft/AirSim-Drone-Racing-Lab repositiry
It simply loads a level and starts a race against a baseline drone
to validate that everything has been set up properly.
"""

>>>>>>> ffc216d (Added necessary docstring/reference)
import airsimdroneracinglab
import threading
=======
>>>>>>> b439a44 (Added various missing type annotations and reformatted docstrings)
import time
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
<<<<<<< HEAD
        self.thread_reset_and_reset_race = threading.Thread(
>>>>>>> cd5dadc (Initial commit)
=======
        self.thread_reset_and_reset_race: threading.Thread = threading.Thread(
>>>>>>> b439a44 (Added various missing type annotations and reformatted docstrings)
            target=self.repeat_timer, args=(self.reset_and_reset_race, 0.09)
        )
        self.is_thread_active = False

<<<<<<< HEAD
<<<<<<< HEAD
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
=======
    def repeat_timer(self, callback, period):
<<<<<<< HEAD
>>>>>>> cd5dadc (Initial commit)
=======
=======
    def repeat_timer(self, callback, period: float) -> None:
>>>>>>> b439a44 (Added various missing type annotations and reformatted docstrings)
        """
        Simple sleep timer.

        Parameters
        ----------
            callback
                Function to call
            period : float
                Repeat interval in seconds
        """
>>>>>>> 89992b9 (Added Docstrings)
        while self.is_thread_active:
            callback()
            time.sleep(period)

<<<<<<< HEAD
<<<<<<< HEAD
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
=======
    def load_level(self, level_name, sleep_sec=2.0):
=======
    def load_level(self, level_name: str, sleep_sec: float = 2.0) -> None:
>>>>>>> b439a44 (Added various missing type annotations and reformatted docstrings)
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
<<<<<<< HEAD
        self.level_name = level_name
>>>>>>> cd5dadc (Initial commit)
=======
        self.level_name: str = level_name
>>>>>>> b439a44 (Added various missing type annotations and reformatted docstrings)
        self.airsim_client.simLoadLevel(self.level_name)
        self.airsim_client.confirmConnection()  # failsafe
        time.sleep(sleep_sec)  # let the environment load completely

<<<<<<< HEAD
<<<<<<< HEAD
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
=======
    def reset(self):
=======
    def reset(self) -> None:
>>>>>>> b439a44 (Added various missing type annotations and reformatted docstrings)
        """Resets Airsim cleint."""
        print(time.time(), "called reset")
        self.airsim_client.reset()

    def reset_race(self) -> None:
        """Resets current race."""
        print(time.time(), "called simResetRace")
        self.airsim_client_2.simResetRace()

<<<<<<< HEAD
    def reset_and_reset_race(self):
<<<<<<< HEAD
>>>>>>> cd5dadc (Initial commit)
=======
=======
    def reset_and_reset_race(self) -> None:
>>>>>>> b439a44 (Added various missing type annotations and reformatted docstrings)
        """Resets Airsim cleint and current race"""
>>>>>>> 89992b9 (Added Docstrings)
        print(time.time(), "called reset, followed by simResetRace")
        self.airsim_client_3.reset()
        self.airsim_client_3.simResetRace()

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
    def start_race(self, tier):
=======
    def start_race(self, tier=1):
=======
    def start_race(self, tier: int = 1) -> None:
>>>>>>> b439a44 (Added various missing type annotations and reformatted docstrings)
        """
        Starts race against baseline drone

        Parameters
        ----------
            tier : int, default=1
                Race tier determining difficulty.
                Tier 1: Planning only; Tier 2: Perception Only; Tier 3: Planning and Perception.
        """
>>>>>>> 89992b9 (Added Docstrings)
        print(time.time(), "called start race")
        self.airsim_client.simStartRace(tier)

<<<<<<< HEAD
    def initialize_drone(self):
<<<<<<< HEAD
>>>>>>> cd5dadc (Initial commit)
=======
=======
    def initialize_drone(self) -> None:
>>>>>>> b439a44 (Added various missing type annotations and reformatted docstrings)
        """
        Initializes user drone, enabling API control and arming the vehicle.
        Sets default values for trajectory tracker gains.
        """
>>>>>>> 89992b9 (Added Docstrings)
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

<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> b439a44 (Added various missing type annotations and reformatted docstrings)
    def start_threads(self) -> None:
        """Starts threads if not already active."""
=======
    def start_threads(self):
<<<<<<< HEAD
>>>>>>> cd5dadc (Initial commit)
=======
        """Starts threads if not already active."""
>>>>>>> 89992b9 (Added Docstrings)
        if not self.is_thread_active:
            self.is_thread_active = True
            self.thread_reset.start()
            self.thread_reset_race.start()
            self.thread_reset_and_reset_race.start()
            print("Started threads")

<<<<<<< HEAD
<<<<<<< HEAD
    def stop_threads(self) -> None:
        """Stops threads if not already stopped."""
=======
    def stop_threads(self):
<<<<<<< HEAD
>>>>>>> cd5dadc (Initial commit)
=======
        """Stops threads if not already stopped"""
>>>>>>> 89992b9 (Added Docstrings)
=======
    def stop_threads(self) -> None:
        """Stops threads if not already stopped."""
>>>>>>> b439a44 (Added various missing type annotations and reformatted docstrings)
        if self.is_thread_active:
            self.is_thread_active = False
            self.thread_reset.join()
            self.thread_reset_race.join()
            self.thread_reset_and_reset_race.join()
            print("Stopped threads.")


if __name__ == "__main__":
    reproducer = ReproduceResetRaceCondition("drone_1")
    reproducer.load_level("Qualifier_Tier_1")
    reproducer.initialize_drone()
    reproducer.start_race(3)
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> cd5dadc (Initial commit)
=======
>>>>>>> 89992b9 (Added Docstrings)
