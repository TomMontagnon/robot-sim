import numpy as np

# === Choix des briques ===
from models.unicycle.model import UnicycleModel, adapt_to_unicycle
from models.unicycle.controllers.lyapunov.kanayama_controller import KanayamaController
from trajectories.circle_traj import CircleTrajectory
from trajectories.straight_traj import StraightTrajectory
from model_free_controllers.geometric_controller import GeometricController
from simulator import Simulator
from visual_engine import animate


def main() -> None:
    # --- Simulation parameters ---
    dt = 0.01
    T = 20.0

    # --- Initial state (repère monde) ---
    x0 = np.array([0.0, 0.0, 0.0])  # x, y, theta

    # --- Instantiate blocks ---
    robot = UnicycleModel()
    traj = CircleTrajectory(radius=5.0, omega=0.3)
    # traj = StraightTrajectory(1.0, [1,1])
    # controller = GeometricController(1, 1, adapt_to_unicycle)
    controller = KanayamaController()
    sim = Simulator(robot, controller, traj, dt)

    # --- Simulation loop ---
    x = x0.copy()
    t = 0.0

    history = []

    while t < T:
        history.append(x.copy())
        x = sim.step(x, t)
        t += dt

    history = np.array(history)
    print("Simulation finished")
    animate(history, traj, dt)


if __name__ == "__main__":
    main()
