class Simulator:
    def __init__(self, robot, controller, adapter, traj, dt=0.01):
        self.robot = robot
        self.controller = controller
        self.adapter = adapter
        self.traj = traj
        self.dt = dt

    def step(self, x, t):
        uc = self.controller.compute(x, self.traj, t)
        u = self.adapter.map(uc)
        dx = self.robot.dynamics(x, u)
        return x + self.dt * dx

