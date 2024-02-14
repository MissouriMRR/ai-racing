"""
This file starts an airsimdroneracinglab environment,
initializes a user drone and a baseline drone,
then autonomously controls the drone using a barebones pathing algorithm.
Many features are hard-coded at times
and the drone's movement is rather choppy,
but this file offers a good starting point for more complex pathing algorithms.
"""

from typing import Callable
import math
import threading
import time
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
        Built-in ADRL async takeoff function.
        Sends moveonspline commands to drone in order to take off.
    give_control_stick_inputs(self, roll, pitch, yaw, z, duration)
        Reads various control stick inputs from command-line interface
        and passes them to the drone for a given amount of time.
    get_gate_pose(gate_index)
        Gets a list of gates from the airsim client,
        then returns the pose of the gate requested in the form of an object.
    get_drone_pose()
        Gets the current position of the drone from the airsim client and returns it as an object.
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
        self.level_name: str
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

    def repeat_timer(self, callback: Callable[[], None], period: float) -> None:
        """
        Simple sleep timer used for resetting threads.

        Parameters
        ----------
            callback : function
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
        self.level_name = level_name
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
        """
        Built-in ADRL async takeoff function.
        Sends moveonspline commands to drone in order to take off.
        """
        self.airsim_client.takeoffAsync().join()

    def give_control_stick_inputs(
        self, roll: float, pitch: float, yaw: float, z: float, duration: float
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
            z : float
                A z (height) value to be passed to the drone given relative to the ground
                Drone automatically travels to this position, smoothing its movement automatically
            duration : float
                Duration for the inputs to be passed to the drone given in seconds

        Notes
        -----
            (from airsimdroneracinglab documenation)
            Roll angle, pitch angle, and yaw angle are given in radians, in the body frame.
            The body frame follows the Front Left Up (FLU) convention, and right-handedness.
        """
        self.airsim_client.moveByRollPitchYawZAsync(
            roll, pitch, yaw, z, duration, vehicle_name=self.drone_name
        )

    def get_gate_pose(self, gate_index: int) -> airsimdroneracinglab.Pose:
        """
        Gets a list of gates from the airsim client,
        then returns the pose of the gate requested in the form of an object.

        Paremeters
        ----------
            gate_index : int
                Index of requested/next gate (starts at 0)

        Returns
        -------
            gate_position : airsimdroneracinglab.Pose
                Position of the next gate given as an object containing a Quaternionr and Vector3r

        """
        gate_list: list[str] = self.airsim_client.simListSceneObjects("Gate.*")
        gate_position: airsimdroneracinglab.Pose = self.airsim_client.simGetObjectPose(
            gate_list[gate_index]
        )
        return gate_position

    def get_drone_pose(self) -> airsimdroneracinglab.Pose:
        """
        Gets the current position of the drone from the airsim client and returns it as an object.

        Returns
        -------
            drone_position : airsimdroneracinglab.Pose
                Current position of drone given as an object containing a Quaternionr and Vector3r
        """
        drone_position: airsimdroneracinglab.Pose = self.airsim_client.simGetObjectPose(
            self.drone_name
        )
        return drone_position

    def get_drone_y_velocity(self) -> float:
        """
        Calculates the drone's Y velocity using the difference in it's position over
        a fixed interval of time.
        """
        interval: float = 0.01

        # Gets drone's positon "interval" number of seconds apart
        old_position: airsimdroneracinglab.Pose = self.get_drone_pose()
        time.sleep(interval)
        new_position: airsimdroneracinglab.Pose = self.get_drone_pose()

        # Gets difference in the new and old position divided by interval to get velocity vectors
        y_vector: float = (new_position.position.y_val - old_position.position.y_val) / interval
        x_vector: float = -((new_position.position.x_val - old_position.position.x_val) / interval)

        # Gets the drone's current objective yaw angle as tuple[roll, pitch, yaw]
        orientation: tuple[float, float, float] = airsimdroneracinglab.utils.to_eularian_angles(
            new_position.orientation
        )
        yaw_angle: float = orientation[2]

        # Gets the y (perpendicular) component of the objective x and y velocity vectors
        x_vector_y_component: float = x_vector / math.cos(yaw_angle)
        y_vector_y_component = y_vector / math.cos(
            yaw_angle - (math.pi / 2)
        )  # -pi/2 to shift -90 degrees

        # Adds the two y componenets together to get the drone's total y velocity
        y_velocity = x_vector_y_component + y_vector_y_component

        if 0 > yaw_angle > -(math.pi / 2):
            y_velocity = -y_velocity
        elif math.pi > yaw_angle > (math.pi / 2):
            y_velocity = -y_velocity

        if not 0.15 > y_velocity > -0.15:
            if y_velocity > 0:
                y_velocity = ((abs(y_velocity) - 1) / (4 * abs(y_velocity))) + 2
            else:
                y_velocity = -(((abs(y_velocity) - 1) / (4 * abs(y_velocity))) + 2)
            print("Drone Y Velocity Adjusted")
        else:
            y_velocity = 0

        print(f"Drone Yaw Angle is: {yaw_angle}")
        print(f"Drone Y Velocity is: {y_velocity}")
        return y_velocity


def generate_vector(
    start_pos: airsimdroneracinglab.Vector3r,
    end_pos: airsimdroneracinglab.Vector3r,
) -> tuple[float, float, float]:
    """
    Takes in two positional vectors, and calculates the vector difference from one to the other.

    Paremeters
    ----------
        start_pos : arisimdroneracinglab.Vector3r
            Starting position for generated vector
        end_pos : airsimdroneracinglab.Vector3r
            Ending position for generated vector

    Returns
    -------
        vector_difference : tuple[float, float, float]
            Vector between the two points given
    """
    x_distance: float = start_pos.x_val - end_pos.x_val
    y_distance: float = -(start_pos.y_val - end_pos.y_val)  # Inverts Y value for proper result
    z_distance: float = start_pos.z_val - end_pos.z_val
    vector_difference: tuple[float, float, float] = (x_distance, y_distance, z_distance)
    print("Vector to Gate:", vector_difference)
    return vector_difference


def generate_yaw_angle(target_vector: tuple[float, float, float]) -> float:
    """
    Calculates the current yaw value required to point the drone towards to its next desired
    position, given a vector pointing from the current drone to the desired new position.

    Parameters
    ----------
        target_vector : tuple[float, float, float]
            Vector from the drone's current position to its desired new position
            (often the desired position is the center of the next gate)

    Returns
    -------
        yaw_angle : float
            Yaw angle needed to point the drone towards it's desired target
    """
    yaw_angle: float = math.atan(target_vector[1] / target_vector[0])
    if yaw_angle > 0:
        yaw_angle += math.pi
    print("Yaw angle:", math.degrees(yaw_angle))
    return yaw_angle


def get_distance_to_target(target_vector: tuple[float, float, float]) -> float:
    """
    Gets the distance from the drone's current location to its target given a vector
    from its current location to its target.

    Parameters
    ----------
        target_vector : tuple[float, float, float]
            Vector from the drone's current position to its desired new position
            (often the desired position is the center of the next gate)

    Returns
    -------
        distance : float
            Distance from the drone to its target
    """
    distance: float = 0
    for distance in target_vector:
        distance += math.pow(distance, 2)
    distance = math.sqrt(distance)
    print(f"Distance to target: {distance}")
    return distance


if __name__ == "__main__":
    # Sets up race, initializes drone and loads level
    reproducer = ReproduceResetRaceCondition("drone_1")
    reproducer.load_level("Soccer_Field_Easy")  # Level name can be changed  - see load_level() args
    reproducer.initialize_drone()
    reproducer.start_race(1)

    drone_pose = reproducer.get_drone_pose()
    drone_orientation = airsimdroneracinglab.utils.to_eularian_angles(drone_pose.orientation)
    reproducer.takeoff()

    for i in range(25):
        INPUT_DURATION: float = 0.1  # Time interval for giving drone commands (in seconds)

        drone_pose = reproducer.get_drone_pose()
        gate_pose = reproducer.get_gate_pose(i)
        drone_orientation = airsimdroneracinglab.utils.to_eularian_angles(drone_pose.orientation)
        gate_orientation = airsimdroneracinglab.utils.to_eularian_angles(gate_pose.orientation)

        distance_to_gate: float = 0
        TARGET_REACHED: bool = False
        while not TARGET_REACHED:
            drone_pose = reproducer.get_drone_pose()
            drone_orientation = airsimdroneracinglab.utils.to_eularian_angles(
                drone_pose.orientation
            )
            vector_to_gate = generate_vector(drone_pose.position, gate_pose.position)
            if math.isnan(vector_to_gate[0]):
                time.sleep(INPUT_DURATION)
                continue
            target_yaw_angle = generate_yaw_angle(vector_to_gate)
            print("Drone Orientation:", drone_orientation)

            drone_y_velocity = reproducer.get_drone_y_velocity()
            drone_roll_angle: float = 0
            if drone_y_velocity > 0:
                drone_roll_angle = -0.075 * drone_y_velocity
            elif drone_y_velocity < 0:
                drone_roll_angle = -0.075 * drone_y_velocity
            print("Roll angle:", drone_roll_angle)
            reproducer.give_control_stick_inputs(
                drone_roll_angle,
                0.02,
                target_yaw_angle,
                gate_pose.position.z_val,
                INPUT_DURATION,
            )
            distance_to_gate = get_distance_to_target(vector_to_gate)
            print()

            if distance_to_gate < 1:
                TARGET_REACHED = True
            time.sleep(INPUT_DURATION)
