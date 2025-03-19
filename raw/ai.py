import numpy as np


class Envirmonment:
    def __init__(self, size=5) -> None:
        self.x = np.random.randint(0, size-1)
        self.y = np.random.randint(0, size-1)
        self.goal = (size, size)
        self.actions = [1, 2, 3, 4]
        self.act = {
            1: (-1, 0),  # Left
            2: (0, 1),  # Right
            3: (-1, 0),  # Top
            4: (0, 1),  # Bottom
        }
        self.n_actions = 4

    # Reward,state,episode_end
    def step(self, action):
        temp_x = self.x + self.act[action][0]
        temp_y = self.y + self.act[action][1]

        if (temp_x >= self.size) or (temp_y >= self.size) or (temp_x < 0) or (temp_y < 0):
            return -1, (self.x, self.y), False
        if (temp_x == (self.size-1) and temp_y == (self.size-1)):
            return 1, (self.x, self.y), True


# 1. Markov Descision Process (MDP)
# 2. Dynamic Programming
# 3. Monte Carlo Methods
# 4. Temporal Difference Methods
# 5. N-step Bootstraping
# 6. Continuous state spaces
# 7. Deep SARSA
# 8. Deep Q-Learning
# 9. Actor Critic (A2C)

# 3. Monte Carlo Methods

# i) On-Policy Monte Carlo

env = Envirmonment()


class AI:
    def __init__(self):
        pass

    def get_object_position(self, object):
        # return np.array([0.4330, 0, 0]).reshape(-1, 1)
        return np.array([0, 0.55, 0.2]).reshape(-1, 1)
