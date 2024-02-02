"""
This file starts an airsimdroneracinglab environment,
initializes a user drone and a baseline drone,
then autonomously controls the drone using a barebones pathing algorithm.
Many features are hard-coded at times
and the drone's movement is rather choppy,
but this file offers a good starting point for more complex pathing algorithms.
"""

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
        then returns the pose of the gate requested in the form of a vector3r.
    get_drone_pose()
        Gets the current position of the drone from the airsim client and returns it as a vector3r.
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

    def get_gate_pose(self, gate_index: int):
        """
        Gets a list of gates from the airsim client,
        then returns the pose of the gate requested in the form of a vector3r.

        Paremeters
        ----------
            gate_index : int
                Index of requested/next gate (starts at 0)

        Returns
        -------
            gate_position : airsimdroneracinglab.vector3r
                Position of the next gate given as a vector3r

        """
        gate_list: list[str] = self.airsim_client.simListSceneObjects("Gate.*")
        gate_position: airsimdroneracinglab.Vector3r = self.airsim_client.simGetObjectPose(
            gate_list[gate_index]
        )
        return gate_position

    def get_drone_pose(self):
        """
        Gets the current position of the drone from the airsim client and returns it as a vector3r

        Returns
        -------
            drone_position : airsimdroneracinglab.Vector3r
        """
        drone_position: airsimdroneracinglab.Vector3r = self.airsim_client.simGetObjectPose(
            self.drone_name
        )
        return drone_position


def generate_vector(start_pos, end_pos):
    x_distance = start_pos.x_val - end_pos.x_val
    y_distance = -(start_pos.y_val - end_pos.y_val)
    z_distance = start_pos.z_val - end_pos.z_val
    vector_to_gate = (x_distance, y_distance, z_distance)
    print("Vector to Gate:", vector_to_gate)
    return vector_to_gate


def generate_yaw_angle(vector_to_gate):
    yaw_angle = math.atan(vector_to_gate[1] / vector_to_gate[0])
    if vector_to_gate[0] > 0:
        yaw_angle += math.pi
    print("Yaw angle:", math.degrees(yaw_angle))
    return yaw_angle


def get_distance_to_gate(vector_to_gate, gate_num):
    distance_to_gate = 0
    for distance in vector_to_gate:
        distance_to_gate += math.pow(distance, 2)
    distance_to_gate = math.sqrt(distance_to_gate)
    print(f"Distance to Gate {gate_num}: {distance_to_gate}")
    return distance_to_gate


def get_drone_y_velocity(reproducer, current_position):
    time.sleep(0.01)
    get_drone_pose = getattr(reproducer, "get_drone_pose")
    new_position = get_drone_pose()
    y_velocity = (new_position.position.y_val - current_position.position.y_val) / 0.01
    x_velocity = -((new_position.position.x_val - current_position.position.x_val) / 0.01)
    drone_orientation = airsimdroneracinglab.utils.to_eularian_angles(new_position.orientation)
    yaw_angle = drone_orientation[2]
    drone_y_velocity = (x_velocity / math.cos(yaw_angle)) + (
        y_velocity / math.cos(yaw_angle - (math.pi / 2))
    )
    if 0 > yaw_angle > -(math.pi / 2):
        drone_y_velocity = -drone_y_velocity
    elif math.pi > yaw_angle > (math.pi / 2):
        drone_y_velocity = -drone_y_velocity
    if not (0.15 > drone_y_velocity > -0.15):
        if drone_y_velocity > 0:
            drone_y_velocity = ((abs(drone_y_velocity) - 1) / (4 * abs(drone_y_velocity))) + 2
        else:
            drone_y_velocity = -(((abs(drone_y_velocity) - 1) / (4 * abs(drone_y_velocity))) + 2)
        print("Drone Y Velocity Adjusted")
    else:
        drone_y_velocity = 0

    print(f"Drone Yaw Angle is: {yaw_angle}")
    print(f"Drone Y Velocity is: {drone_y_velocity}")
    return drone_y_velocity


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
        input_duration = 0.1  # Time interval for giving drone commands (in seconds)

        drone_pose = reproducer.get_drone_pose()
        gate_pose = reproducer.get_gate_pose(i)
        drone_orientation = airsimdroneracinglab.utils.to_eularian_angles(drone_pose.orientation)
        gate_orientation = airsimdroneracinglab.utils.to_eularian_angles(gate_pose.orientation)

        distance_to_gate = 0
        location_reached: bool = False
        while not location_reached:
            drone_pose = reproducer.get_drone_pose()
            drone_orientation = airsimdroneracinglab.utils.to_eularian_angles(
                drone_pose.orientation
            )
            vector_to_gate = generate_vector(drone_pose.position, gate_pose.position)
            if math.isnan(vector_to_gate[0]):
                time.sleep(input_duration)
                continue
            yaw_angle = generate_yaw_angle(vector_to_gate)
            print("Drone Orientation:", drone_orientation)

            y_velocity = get_drone_y_velocity(reproducer, drone_pose)
            if y_velocity > 0:
                roll_angle = -0.075 * y_velocity
            elif y_velocity < 0:
                roll_angle = -0.075 * y_velocity
            else:
                roll_angle = 0
            print("Roll angle:", roll_angle)
            reproducer.give_control_stick_inputs(
                roll_angle, 0.02, yaw_angle, gate_pose.position.z_val, input_duration
            )
            distance_to_gate = get_distance_to_gate(vector_to_gate, i)
            print()

            if distance_to_gate < 1:
                location_reached = True
            time.sleep(input_duration)
