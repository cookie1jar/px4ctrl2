"""Smooth position/yaw references with sampled speed and acceleration bounds."""
import math
import numpy as np


class GoalTrajectory:
    def __init__(self, start, velocity, acceleration, goal, max_speed=0.4, max_accel=0.3):
        start, velocity, acceleration, goal = map(
            lambda value: np.asarray(value, dtype=float), (start, velocity, acceleration, goal))
        if any(value.shape != (4,) or not np.isfinite(value).all()
               for value in (start, velocity, acceleration, goal)):
            raise ValueError('position/yaw references must contain four finite values')
        if not all(math.isfinite(v) and v > 0 for v in (max_speed, max_accel)):
            raise ValueError('speed and acceleration limits must be positive')
        goal = goal.copy()
        goal[3] = start[3] + math.atan2(math.sin(goal[3]-start[3]), math.cos(goal[3]-start[3]))
        delta = goal-start
        duration = max(2., 1.875*np.linalg.norm(delta[:3])/max_speed,
                       math.sqrt(5.78*np.linalg.norm(delta[:3])/max_accel),
                       1.875*abs(delta[3])/0.5)
        for _ in range(30):
            t = duration
            self.coefficients = np.array([
                start, velocity, acceleration/2,
                (20*delta-12*velocity*t-3*acceleration*t*t)/(2*t**3),
                (-30*delta+16*velocity*t+3*acceleration*t*t)/(2*t**4),
                (12*delta-6*velocity*t-acceleration*t*t)/(2*t**5)])
            self.duration = duration
            samples = [self.sample(t) for t in np.linspace(0, duration, 201)]
            if (max(np.linalg.norm(v[:3]) for _, v, _ in samples) <= max_speed*1.001
                    and max(np.linalg.norm(a[:3]) for _, _, a in samples) <= max_accel*1.001
                    and max(abs(v[3]) for _, v, _ in samples) <= 0.501):
                return
            duration *= 1.25
        raise ValueError('cannot construct trajectory within limits')

    def sample(self, elapsed):
        t = min(max(float(elapsed), 0.), self.duration)
        c = self.coefficients
        p = sum(c[i]*t**i for i in range(6))
        v = sum(i*c[i]*t**(i-1) for i in range(1, 6))
        a = sum(i*(i-1)*c[i]*t**(i-2) for i in range(2, 6))
        return p, v, a
