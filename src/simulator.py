class Simulator:
    def __init__(self, robot, controller, traj, dt=0.01):
        self.robot = robot
        self.controller = controller
        self.traj = traj
        self.dt = dt

    def step(self, x, t):
        u = self.controller.compute(x, self.traj, t)
        dx = self.robot.dynamics(x, u)
        return x + self.dt * dx

