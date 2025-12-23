from api.abstract_robot_model import BasicRobotState, AbstractRobotModel
from api.abstract_controller import AbstractController
from api.abstract_traj import AbstractTrajectory


class Simulator:
    def __init__(
        self,
        model: AbstractRobotModel,
        controller: AbstractController,
        traj: AbstractTrajectory,
        dt: float = 0.01,
    ) -> None:
        self.model = model
        self.controller  = controller
        self.traj = traj
        self.dt = dt

    def step(self, x: BasicRobotState, t: float) -> None:
        u = self.controller.compute(x, self.traj, t)
        dx = self.model.dynamics(x, u)
        x.update(dx, self.dt)
