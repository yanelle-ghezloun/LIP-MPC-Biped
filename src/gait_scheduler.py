import numpy as np


class GaitScheduler:

    def __init__(self, step_length=0.2, step_duration=0.8, foot_size=0.1, dt=0.01):
        self.step_length = step_length
        self.step_duration = step_duration
        self.foot_size = foot_size
        self.dt = dt
        self.steps_per_phase = int(step_duration / dt)
        self.footsteps = []
        self._generate_footsteps(n_steps=8)
        self.current_step = 0
        self.step_counter = 0

    def _generate_footsteps(self, n_steps):
        x = 0.0
        for i in range(n_steps):
            side = 'left' if i % 2 == 0 else 'right'
            self.footsteps.append((x, side))
            x += self.step_length

    def get_current_support(self):
        if self.current_step >= len(self.footsteps):
            foot_x = self.footsteps[-1][0]
        else:
            foot_x = self.footsteps[self.current_step][0]
        zmp_min = foot_x - self.foot_size / 2
        zmp_max = foot_x + self.foot_size / 2
        return zmp_min, zmp_max

    def get_support_sequence(self, horizon):
        zmp_min = np.zeros(horizon)
        zmp_max = np.zeros(horizon)
        step_idx = self.current_step
        step_count = self.step_counter
        for k in range(horizon):
            if step_idx >= len(self.footsteps):
                foot_x = self.footsteps[-1][0]
            else:
                foot_x = self.footsteps[step_idx][0]
            zmp_min[k] = foot_x - self.foot_size / 2
            zmp_max[k] = foot_x + self.foot_size / 2
            step_count += 1
            if step_count >= self.steps_per_phase:
                step_count = 0
                step_idx += 1
        return zmp_min, zmp_max

    def get_com_reference(self, horizon, current_x):
        x_ref = np.zeros((2, horizon + 1))
        step_idx = self.current_step
        step_count = self.step_counter
        for k in range(horizon + 1):
            if step_idx >= len(self.footsteps):
                foot_x = self.footsteps[-1][0]
            else:
                foot_x = self.footsteps[step_idx][0]
            x_ref[0, k] = foot_x
            x_ref[1, k] = self.step_length / self.step_duration
            step_count += 1
            if step_count >= self.steps_per_phase:
                step_count = 0
                step_idx += 1
        return x_ref

    def update(self):
        self.step_counter += 1
        if self.step_counter >= self.steps_per_phase:
            self.step_counter = 0
            self.current_step += 1

    def is_done(self):
        return self.current_step >= len(self.footsteps)

    def get_footsteps(self):
        return self.footsteps