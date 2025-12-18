import numpy as np

# === Choix des briques ===
from robot_models import UnicycleModel
from adapters import UnicycleAdapter
from trajectories import CircleTrajectory, StraightTrajectory
from controllers import GeometricController
from simulator import Simulator
from visual_engine import animate

def main() -> None:

    # --- Simulation parameters ---
    dt = 0.01
    T  = 20.0

    # --- Initial state (repère monde) ---
    x0 = np.array([0.0, 0.0, 0.0])  # x, y, theta

    # --- Instantiate blocks ---
    robot = UnicycleModel()
    adapter = UnicycleAdapter()
    traj = CircleTrajectory(R=100.0, omega=0.3)
    # traj = StraightTrajectory(1.0, [1,1])
    controller = GeometricController(1, 1)
    sim = Simulator(robot, controller, adapter, traj, dt)

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

