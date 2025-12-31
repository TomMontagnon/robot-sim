import numpy as np
from dataclasses import astuple

# === Choix des briques ===
from models.unicycle.model import (
    UnicycleModel,
    UnicycleM2Model,
    UnicycleState,
)
from models.unicycle.controllers.lyapunov.kanayama1990 import KanayamaController
from models.unicycle.controllers.lyapunov.m2_par import M2parController
from trajectories.circle_traj import CircleTrajectory
from trajectories.straight_traj import StraightTrajectory
from model_free_controllers.geometric_controller import GeometricController
from simulator import Simulator
from visual_engine import animate


M_FREE = UnicycleModel()


def main() -> None:
    # --- Simulation parameters ---
    dt = 0.01
    T = 10

    # --- Initial state (repère monde) ---
    x = UnicycleState(0, 0, 0)
    t = 0.0

    # --- traj ---
    traj = CircleTrajectory(radius=5.0, omega=0.3)
    # traj = StraightTrajectory(1, [0, 1])

    # =======================CONTROLLER TO CHOSE==================================
    # KANAYAMA DEMO
    controller = KanayamaController()

    # M2 COURS DEMO
    # controller = M2parController()

    # GEOM DEMO
    # controller = GeometricController(1, 1, M_FREE.adapt_input, M_FREE.adapt_output)

    # =======================================================

    model = (
        M_FREE if isinstance(controller, GeometricController) else controller.model()
    )
    if not model.is_std:
        x.theta += np.pi / 2

    sim = Simulator(model, controller, traj, dt)

    history = []

    while t < T:
        history.append(np.array(astuple(x)))
        sim.step(x, t)
        t += dt

    history = np.array(history)
    # Realigner avec le vecteur vitesse si pas standard
    if not model.is_std:
        history[:, 2] -= np.pi / 2
    print("Simulation finished")
    animate(history, traj, dt)


if __name__ == "__main__":
    main()
