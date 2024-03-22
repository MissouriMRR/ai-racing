# AI Structure

- The main form of the AI used in this project has 3 main parts, the Agent, the Simulation, and the Model.

- **Important Defenitions**:
  - Reward
    - pass gate: `+10`
    - is disqualified: `-10`
    - else: `0`
  - Action [roll, pitch, yaw, throttle] (size 4)
    - [`-π/2` - `π/2`, `-π/2` - `π/2`, `-π/2` - `π/2`, `0` - `1`]
  - State (size 12)

    ```python
    [drone x, drone y, drone z
    drone roll, drone pitch, drone yaw
    gate x, gate y, gate z
    gate roll, gate pitch, gate yaw]
    ```

  - Deep Q Learning
    - Q Value = Quality of the action
      1. Initial Q Value (`= init model`)
      2. Choose action (`model.predict(state)`) (or a random move)
          - Tradeoff between exploration and exploitation
      3. Perform action
      4. Measure reward
      5. Update Q value (+ train model)
      6. Repeat steps 2-5

  - Bellman Equation
    - `NewQ(s, a) = Q(s, a) + α[R(s, a) + γmax(Q'(s', a') - Q(s, a))]`
      - Where:
        - `NewQ(s, a)` is the new `Q` value for that state and that action
        - `Q(s, a)` is the current `Q` value
        - `α` is the learning rate
        - `R(s, a)` is the reward fro taking that action at that state
        - `γ` is the discount rate
        - `Q'(s', a')` is the maximum expected future reward given the new `s'` and all possible actions at the new state
    - Simlified:
      - `Q = model.predict(state0)`
        - where `state0` is the initial state of the simulation
      - `Qnew = R + γ * max(Q(state1))`
        - where `R` is reward, `γ` is the discount rate, and `max(Q(state1))` is the maximum `Q` value of the next state

  - Loss function
    - `loss = (Qnew - Q)^2`

## Model (PyTorch)

- IDK what it does yet tbh
- **Critical Components**
  - `linearQNet(DQN)`
    - `model.predict(state)` -> `action`

## Simulation (AirSim Drone Racing Lab)

- Everything needed to perform the simulation resides within the `simulation.py` script.
  - This script repeatedly runs a simulation of a drone race on a level, used to train the machine learning model.
- **Critical Components**
  - `simulate_step(action)` -> `reward`, `is_disqualified`, `race_time`

## Agent

- Presides over both the simulation and the model, bridging the gap between the two.

- **Critical Components**
  - Simulation
  - Model
  - Training
    - `state = get_state(simulation)`
    - `action = get_inputs(state)`
      - `model.predict`
    - `reward, is_disqualified, race_time = simulation.simulate_step(action)`
    - `new_state = get_state(simulation)`
    - remember
      - `model.train()`
