"""
AI training agent
"""

import random
from collections import deque
import math
import torch
import numpy as np
from simulation import ReproduceResetRaceCondition  # TODO
import airsimdroneracinglab

MAX_MEMORY: int = 100_000
BATCH_SIZE: int = 1000
LR: float = 0.001  # Learning Rate


class Agent:
    """
    Agent Class

    Attributes
    ----------

    Methods
    -------

    """

    def __init__(self):
        self.num_simulations = 0
        self.epsilon = 0  # controls randomness
        self.gamma = 0  # discount rate
        self.memory = deque(maxlen=MAX_MEMORY)  # removes overflow from the left
        self.model = None  # Todo
        self.trainer = None  # Todo

    def get_state(self, simulation: ReproduceResetRaceCondition) -> np.array:
        drone_pose: airsimdroneracinglab.Pose = simulation.get_drone_pose()
        drone_position: airsimdroneracinglab.Vector3r = drone_pose.position
        drone_orientation: tuple[float, float, float] = (
            airsimdroneracinglab.utils.to_eularian_angles(drone_pose.orientation)
        )

        drone_x: float = drone_position.x_val
        drone_y: float = drone_position.y_val
        drone_z: float = drone_position.z_val
        drone_roll: float = drone_orientation[0]
        drone_pitch: float = drone_orientation[1]
        drone_yaw: float = drone_orientation[2]

        gate_pose = airsimdroneracinglab.Pose = simulation.get_gate_pose(
            simulation.get_last_gate_passed() + 1
        )
        gate_position: airsimdroneracinglab.Vector3r = gate_pose.position
        gate_orientation: tuple[float, float, float] = (
            airsimdroneracinglab.utils.to_eularian_angles(gate_pose.orientation)
        )

        gate_x: float = gate_position.x_val
        gate_y: float = gate_position.y_val
        gate_z: float = gate_position.z_val
        gate_roll: float = gate_orientation[0]
        gate_pitch: float = gate_orientation[1]
        gate_yaw: float = gate_orientation[2]

        state: float[12] = [
            drone_x,
            drone_y,
            drone_z,
            drone_roll,
            drone_pitch,
            drone_yaw,
            gate_x,
            gate_y,
            gate_z,
            gate_roll,
            gate_pitch,
            gate_yaw,
        ]

        return np.array(state, dtype=float)

    def remember(self, state, action, reward, next_state, is_done):
        self.memory.append((state, action, reward, next_state, is_done))

    def train_long_memory(self):
        if len(self.memory) > BATCH_SIZE:
            mini_sample = random.sample(
                self.memory, BATCH_SIZE
            )  # returns list of tuples
        else:
            mini_sample = self.memory

        states, actions, rewards, next_states, is_dones = zip(*mini_sample)
        self.trainer.train_step(states, actions, rewards, next_states, is_dones)

    def train_short_memory(self, state, action, reward, next_state, is_done):
        self.trainer.train_step(state, action, reward, next_state, is_done)

    def get_action(self, state):
        # random moves
        self.epsilon = (
            80 - self.num_simulations
        )  # adjusting this value will affect how many explitative moves are made
        final_move = [0, 0, 0, 0.5987]
        if random.randint(0, 200) < self.epsilon:
            for i in range(len(final_move)):
                if i == 3:
                    final_move[i] = random.uniform(0, 1)
                else:
                    final_move[i] = random.uniform(-math.pi / 2, math.pi / 2)
        else:
            state0 = torch.tensor(state, dtype=torch.float)
            prediction: float[4] = self.model.predict(state0)
            for i in range(len(prediction)):
                if i == 3:
                    final_move[i] = max(min(prediction[i], 1), 0)
                else:
                    final_move[i] = max(min(prediction[i], math.pi / 2), -math.pi / 2)
        return final_move


def train():
    plot_times = []
    plot_mean_times = []
    race_time = 0
    record = 0
    agent: Agent = Agent()
    simulation = ReproduceResetRaceCondition()
    while True:
        # get the old state
        state_old = agent.get_state(simulation)  # TODO

        # get move
        final_move = agent.get_action(state_old)

        # perform move and get new state
        reward, is_done, race_time = simulation.simulate_step()  # TODO
        state_new = agent.get_state(simulation)  # TODO

        # train short memory
        agent.train_short_memory(state_old, final_move, reward, state_new, is_done)

        # remember
        agent.remember(state_old, final_move, reward, state_new, is_done)

        if is_done:
            # train long memory, plot results
            simulation.reset_and_reset_race()  # TODO
            agent.num_simulations += 1
            agent.train_long_memory()

            if race_time < record:
                record = race_time
                # TODO: agent.model.save()

            print(
                "Simulation",
                agent.num_simulations,
                "Time:",
                race_time,
                "Record:",
                record,
            )

            # TODO: make plot


if __name__ == "__main__":
    train()
