"""
This file starts an airsimdroneracinglab environment,
initializes a user drone and a baseline drone,
then autonomously controls the drone using a barebones pathing algorithm.
Movements are controlled with PID loops.
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
        Passes low level inputs to the drone to take of rapidly upward and foreword.
        Inputs last exactly 1 second during which the program sleeps before continuing.
    give_control_stick_inputs(self, roll, pitch, yaw, z, duration)
        Reads various control stick inputs from command-line interface
        and passes them to the drone for a given amount of time.
    get_gate_pose(gate_index)
        Gets a list of gates from the airsim client,
        then returns the pose of the gate requested in the form of an object.
    get_drone_pose()
        Gets the current position of the drone from the airsim client and returns it as an object.
    get_drone_velocity_components()
        Calculates the perpendicular x, y, and z components of the drone using the change
        in the drone's position over time and trigonometric functions.
    get_num_gates()
        Gets the total number of gates in the active level from the airsim client.
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

    def takeoff(self, orientation: tuple[float, float, float]) -> None:
        """
        Passes low level inputs to the drone to take of rapidly upward and foreword.
        Inputs last exactly 1 second during which the program sleeps before continuing.

        Args:
        -----
            drone_orientation : tuple[roll, pitch, yaw]
                All given in respect to drone's frame following FLU convention, in radians.
        """
        self.airsim_client.moveByRollPitchYawThrottleAsync(
            0, 0, orientation[2] + math.pi, 0.8, 1, vehicle_name=self.drone_name
        )
        time.sleep(1)

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
                A throttle value from 0.0 to 1.0
                (Neutral/Hover is about 0.5938 for the default drone)
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

    def get_drone_velocity_components(
        self,
        yaw_angle: float,
        x_velocity_deadzone: float = 0.05,
        y_velocity_deadzone: float = 0.05,
        z_velocity_deadzone: float = 0.05,
    ) -> tuple[float, float, float]:
        """
        Calculates the perpendicular x, y, and z components of the drone using gps data
        and trigonometric functions.

        Parameters
        ----------
            yaw_angle : float
                The current yaw angle of the drone
            x_velocity_deadzone : float
                Values between this value +- returned as 0 (defaults to .05)
            y_velocity_deadzone : float
                Values between this value +- returned as 0 (defaults to .05)
            z_velocity_deadzone : float
                Values between this value +- returned as 0 (defaults to .05)

        Returns
        -------
            velocity_components : tuple[float, float, float]
                Contains the drones current x, y, and z velocity components, respectively
        """

        # Gets drone's velocity from GPS data with respect to the ground
        velocity_vector: airsimdroneracinglab.Vector3r = (
            self.airsim_client.getGpsData().gnss.velocity
        )

        # Performs trig operations to translate vectors to be perpendicular with respect to drone
        x_velocity: float = (velocity_vector.y_val * math.cos(yaw_angle + (math.pi / 2))) + (
            velocity_vector.x_val * math.cos(yaw_angle)
        )

        y_velocity: float = (velocity_vector.y_val * math.sin(yaw_angle + (math.pi / 2))) + (
            velocity_vector.x_val * math.sin(yaw_angle)
        )

        z_velocity: float = -velocity_vector.z_val

        # if y velocity is inside deadzone set it to 0
        if x_velocity_deadzone > x_velocity > -x_velocity_deadzone:
            x_velocity = 0
        if y_velocity_deadzone > y_velocity > -y_velocity_deadzone:
            y_velocity = 0
        if z_velocity_deadzone > z_velocity > -z_velocity_deadzone:
            z_velocity = 0

        print(f"Drone X Velocity is: {x_velocity}")
        print(f"Drone Y Velocity is: {y_velocity}")
        print(f"Drone Z Velocity is: {z_velocity}")

        velocity_components: tuple[float, float, float] = (
            x_velocity,
            y_velocity,
            z_velocity,
        )

        return velocity_components

    def get_num_gates(self) -> int:
        """
        Gets the total number of gates in the active level from the airsim client.

        Returns
        -------
            num_gates : int
                The number of gates present on the active level
        """
        gate_list: list[str] = self.airsim_client.simListSceneObjects("Gate.*")
        num_gates = len(gate_list)
        return num_gates


class PID:
    """
    Contains various functions to implement a pid loop.

    Attributes
    ----------
    Kp : float
        The proportional coefficient used in PID calculation (defaults to 1).
    Ki : float
        The integral coefficient used in PID calculation (defaults to 0).
    Kd : float
        The derivative coefficient used in PID calculation (defaults to 0).
    max_output : float
        The maximum possible output obtained from PID calculation (defaults to 1).
        The opposite of this is also used as the minimum possible output.
    sample_rate : float
        Time in between PID calculations for calculation with respect to time (defaults to .1).
    output : float
        The output value of the PID loop.
    target : float
        The target output of the PID loop.
    accumulator : float
        Accumulates based on the error value over time, used in integral calculation.
    last_reading : float
        Holds the value of the last input value to the PID loop.

    Methods
    -------
    set_target(target: float)
        Sets the PID loop to a new target, clears accumulator.
    adjust_output(current_value: float)
        Calculates output of PID loop using given current value and predefined coefficients.
    """

    def __init__(
        self,
        kp: float = 1,
        ki: float = 0,
        kd: float = 0,
        max_output: float = 1,
        sample_rate: float = 0.1,
    ) -> None:
        self.kp: float = kp
        self.ki: float = ki
        self.kd: float = kd
        self.max_output: float = max_output
        self.sample_rate: float = sample_rate

        self.output: float = 0
        self.target: float = 0
        self.accumulator: float = 0
        self.last_reading: float = 0

    def set_target(self, target: float) -> None:
        """
        Sets the PID loop to a new target, clears accumulator.

        Parameters
        ----------
            target : float
                The target output of the PID loop.
        """
        self.accumulator = 0
        self.target = target

    def adjust_output(self, current_value: float) -> float:
        """
        Calculates output of PID loop using given current value and predefined coefficients.

        Parameters
        ----------
            current_value : float
                The current value of the parameter being optimized.

        Returns
        -------
            self.output : float
                Output value of PID calculations
        """
        error: float = self.target - current_value

        self.accumulator += error

        self.output = (
            self.kp * error
            + self.ki * self.accumulator
            + self.kd * (current_value - self.last_reading) / self.sample_rate
        )

        if self.output > self.max_output:
            self.output = self.max_output
        elif self.output < -self.max_output:
            self.output = -self.max_output

        self.last_reading = current_value

        return self.output


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
    if target_vector[0] > 0:
        yaw_angle += math.pi
    print("Target yaw angle:", math.degrees(yaw_angle))
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
    for component in target_vector:
        distance += math.pow(component, 2)
    distance = math.sqrt(distance)
    print(f"Distance to target: {distance}")
    return distance


if __name__ == "__main__":
    # Sets up race, initializes drone, loads level, and takes off
    reproducer = ReproduceResetRaceCondition("drone_1")
    reproducer.load_level("Soccer_Field_Easy")  # Level name can be changed - see load_level()
    NUM_GATES = 12  # There are 12 gates on the default level "Soccer_Field_Easy"
    reproducer.initialize_drone()
    reproducer.start_race(1)

    # Gets drone's orientation to take off in the right direction
    drone_pose: airsimdroneracinglab.Pose = reproducer.get_drone_pose()
    drone_orientation = airsimdroneracinglab.utils.to_eularian_angles(drone_pose.orientation)
    reproducer.takeoff(drone_orientation)

    # Gets the number of gates in the active level
    NUM_GATES = reproducer.get_num_gates()
    print("There are ", NUM_GATES, " gates present.")

    throttle_PID: PID = PID(kp=1, max_output=1)
    z_velocity_PID: PID = PID(kp=0.2, max_output=0.8)
    pitch_PID: PID = PID(kp=0.05, max_output=math.pi / 9)
    pitch_PID.set_target(6)
    roll_PID: PID = PID(kp=0.15, max_output=(4 * math.pi) / 9)
    roll_PID.set_target(0)

    # Iterates through each gate in the level
    for next_gate in range(NUM_GATES):
        INPUT_DURATION: float = 0.1  # Time interval for giving drone commands (in seconds)
        drone_pitch_angle: float = (
            0.02  # Currently a constant, should be calculated with a PID loop
        )

        # Gets next gate position and orientation
        gate_pose = reproducer.get_gate_pose(next_gate)
        gate_orientation = airsimdroneracinglab.utils.to_eularian_angles(gate_pose.orientation)

        throttle_PID.set_target(gate_pose.position.z_val)

        # Gives the drone inputs to move it towards the next gate until it arrives
        distance_to_gate: float = 10000
        while distance_to_gate > 1:
            # Gets the drone's current position and orientation
            drone_pose = reproducer.get_drone_pose()
            drone_orientation = airsimdroneracinglab.utils.to_eularian_angles(
                drone_pose.orientation
            )
            print("Drone Orientation:", drone_orientation)

            # Generates vector to next gate, then verifies that the vector is a number
            vector_to_gate = generate_vector(drone_pose.position, gate_pose.position)
            if math.isnan(vector_to_gate[0]):
                time.sleep(INPUT_DURATION)
                continue

            # Calculates the drone's current velocity as 3 vectors perpendicular to its frame
            drone_velocity_components: tuple[
                float, float, float
            ] = reproducer.get_drone_velocity_components(-drone_orientation[2])
            print("Current drone yaw angle:", -math.degrees(drone_orientation[2]))
            drone_x_velocity: float = drone_velocity_components[0]
            drone_y_velocity: float = drone_velocity_components[1]
            drone_z_velocity: float = drone_velocity_components[2]

            # Calculates various angles/inputs needed to get the drone through the next gate
            drone_z_velocity_input: float = -throttle_PID.adjust_output(drone_pose.position.z_val)
            z_velocity_PID.set_target(drone_z_velocity_input)
            drone_throttle_input: float = 0.5938 + z_velocity_PID.adjust_output(drone_z_velocity)
            print("Throttle: ", drone_throttle_input)
            drone_roll_angle = roll_PID.adjust_output(drone_y_velocity)
            print("Roll: ", math.degrees(drone_roll_angle))
            drone_pitch_angle = pitch_PID.adjust_output(drone_x_velocity)
            print("Pitch: ", drone_pitch_angle)

            if distance_to_gate > 2.5:
                target_yaw_angle = generate_yaw_angle(vector_to_gate)

            # Sends inputs to the drone
            reproducer.give_control_stick_inputs(
                drone_roll_angle,
                drone_pitch_angle,
                target_yaw_angle,
                drone_throttle_input,
                INPUT_DURATION,
            )
            # Updates distance to the gate, and sleeps to let the drone execute inputs
            distance_to_gate = get_distance_to_target(vector_to_gate)
            print()
            time.sleep(INPUT_DURATION)
    print("\nCourse Completed!!")
