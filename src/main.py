import numpy as np
from dataclasses import astuple

# === Choix des briques ===
from models.unicycle.model import (
    UnicycleModel,
    UnicycleVariantModel,
    UnicycleState,
    adapt_to_unicycle,
)
from models.unicycle.controllers.lyapunov.kanayama1990 import KanayamaController
from models.unicycle.controllers.lyapunov.m2_par import M2parController
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
    x = UnicycleState(0, 0, np.pi / 2)
    t = 0.0

    # --- Instantiate blocks ---
    traj = CircleTrajectory(radius=5.0, omega=0.3)

    # =======================A CHOISIR==================================
    # KANAYAMA DEMO
    # model = UnicycleModel()
    # controller = KanayamaController()

    # M2 COURS DEMO
    model = UnicycleVariantModel()
    controller = M2parController()
    # =======================================================
    sim = Simulator(model, controller, traj, dt)

    history = []

    while t < T:
        history.append(np.array(astuple(x)))
        sim.step(x, t)
        t += dt

    history = np.array(history)
    print("Simulation finished")
    animate(history, traj, dt)


if __name__ == "__main__":
    main()
