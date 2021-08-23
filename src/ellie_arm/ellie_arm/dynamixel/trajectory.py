from ellie_arm.dynamixel.sub_threading import ModifiedLoopThread
import numpy as np
import collections
import time

class MinimumJerkTrajectory(object):
    def __init__(self, initial, final, duration, init_vel=0.0, init_acc=0.0, final_vel=0.0, final_acc=0.0):
        self.initial = initial
        self.final = final
        self.duration = duration
        self.init_vel = init_vel
        self.init_acc = init_acc
        self.final_vel = final_vel
        self.final_acc = final_acc

        self.durations = [0, duration]
        self.finals = [final]

        self.compute()

    def compute(self):  # N'Guyen's magic
        a0 = self.initial
        a1 = self.init_vel
        a2 = self.init_acc / 2.0
        d = lambda x: self.duration ** x

        A = np.array([[d(3), d(4), d(5)], [3 * d(2), 4 * d(3), 5 * d(4)], [6 * d(1), 12 * d(2), 20 * d(3)]])
        B = np.array([self.final - a0 - (a1 * d(1)) - (a2 * d(2)), self.final_vel - a1 - (2 * a2 * d(1)), self.final_acc - (2 * a2)])
        X = np.linalg.solve(A, B)

        self.other_gen = None

        self._mylambda = lambda x: a0 + a1 * x + a2 * x ** 2 + X[0] * x ** 3 + X[1] * x ** 4 + X[2] * x ** 5

        self._generators = [self._mylambda]

    def get_value(self, t):
        return self._mygenerator[-1](t)

    def domain(self, x):
        if not isinstance(x, collections.Iterable):
            x = np.array([x])

        return np.array([
            self.durations[0] <= xi < self.durations[1]
            for xi in x
        ])

    def test_domain(self, x):
        return [((np.array(x) >= self.durations[i])) for i in range(len(self.durations) - 1)]

    def fix_input(self, x):
        return x if isinstance(x, collections.Iterable) else np.array([0, x])

    def get_generator(self):
        return lambda x: np.piecewise(x, self.domain(x), [self._generators[j] for j in range(len(self._generators))] + [self.finals[-1]])

class MinJerkTrajectory(ModifiedLoopThread):
    def __init__(self, motor, position, duration,frequency):
        super().__init__(frequency)

        self.motor = motor
        self.goal = position
        self.duration = duration

    def setup(self):
        """ Setup method call just before the run. """
        if self.duration < np.finfo(float).eps:             #1.1920929e-07
            self.motor.goal_position = self.goal
            self.stop()
        else:
            self.trajectory = MinimumJerkTrajectory(self.motor.present_position, self.goal, self.duration).get_generator()
        self.t0 = time.time()
    @property
    def elapsed_time(self):
        return time.time() - self.t0
    def update(self):
        if np.finfo(float).eps < self.duration >self.elapsed_time:
            self.motor.goal_position = self.trajectory(self.elapsed_time)
        else:
            self.stop(wait=False)


class SinusTrajectory(object):
    def __init__(self, initial, final, duration, init_vel=0.0, init_acc=0.0, final_vel=0.0, final_acc=0.0):
        self.initial = initial
        self.final = final
        self.duration = duration
        self.init_vel = init_vel
        self.init_acc = init_acc
        self.final_vel = final_vel
        self.final_acc = final_acc

        self.durations = [0, duration]
        self.finals = [final]

        self.compute()

    def compute(self):  # N'Guyen's magic
        a0 = self.initial
        a1 = self.init_vel
        a2 = self.init_acc / 2.0
        d = lambda x: self.duration ** x

        A = np.array([[d(3), d(4), d(5)], [3 * d(2), 4 * d(3), 5 * d(4)], [6 * d(1), 12 * d(2), 20 * d(3)]])
        B = np.array([self.final - a0 - (a1 * d(1)) - (a2 * d(2)), self.final_vel - a1 - (2 * a2 * d(1)), self.final_acc - (2 * a2)])
        X = np.linalg.solve(A, B)

        self.other_gen = None

        self._mylambda = lambda x: a0 + a1 * x + a2 * x ** 2 + X[0] * x ** 3 + X[1] * x ** 4 + X[2] * x ** 5

        self._generators = [self._mylambda]

    def get_value(self, t):
        return self._mygenerator[-1](t)

    def domain(self, x):
        if not isinstance(x, collections.Iterable):
            x = np.array([x])

        return np.array([
            self.durations[0] <= xi < self.durations[1]
            for xi in x
        ])

    def test_domain(self, x):
        return [((np.array(x) >= self.durations[i])) for i in range(len(self.durations) - 1)]

    def fix_input(self, x):
        return x if isinstance(x, collections.Iterable) else np.array([0, x])

    def get_generator(self):
        return lambda x: np.piecewise(x, self.domain(x), [self._generators[j] for j in range(len(self._generators))] + [self.finals[-1]])

class SinusTrajectory(ModifiedLoopThread):
    def __init__(self, motor, position, duration,frequency):
        super().__init__(frequency)

        self.motor = motor
        self.goal = position
        self.duration = duration

    def setup(self):
        """ Setup method call just before the run. """
        if self.duration < np.finfo(float).eps:             #1.1920929e-07
            self.motor.goal_position = self.goal
            self.stop()
        else:
            self.trajectory = MinimumJerkTrajectory(self.motor.present_position, self.goal, self.duration).get_generator()
        self.t0 = time.time()
    @property
    def elapsed_time(self):
        return time.time() - self.t0
    def update(self):
        if np.finfo(float).eps < self.duration >self.elapsed_time:
            self.motor.goal_position = self.trajectory(self.elapsed_time)
        else:
            self.stop(wait=False)