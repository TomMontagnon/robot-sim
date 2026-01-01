import numpy as np
from dataclasses import astuple
from copy import deepcopy

# === Choix des briques ===
from models.unicycle.model import (
    UnicycleModel,
    UnicycleM2Model,
    UnicycleState,
)
from models.unicycle.controllers.lyapunov.kanayama1990 import KanayamaController
from models.unicycle.controllers.lyapunov.samson1990 import SamsonController
from models.unicycle.controllers.lyapunov.m2_par import M2parController
from trajectories.circle_traj import CircleTrajectory
from trajectories.straight_traj import StraightTrajectory
from trajectories.composite_traj import CompositeTrajectory
from model_free_controllers.geometric_controller import GeometricController
from simulator import Simulator
from visual_engine import animate


M_FREE = UnicycleModel()


def main() -> None:
    # --- Simulation parameters ---
    dt = 0.01
    T = 20

    # --- Initial state (repère monde) ---
    x_ini = UnicycleState(0, 0, 0)
    t_ini = 0.0

    # --- traj ---
    # traj = CircleTrajectory(radius=5.0, omega=0.3)
    # traj = StraightTrajectory(1, [0, 1])
    traj = CompositeTrajectory()

    traj.add_fragment(StraightTrajectory(speed=1, direc=[0, 1]), duration=5)
    traj.add_fragment(CircleTrajectory(radius=3, omega=-0.5, angle_offset=np.pi), duration=np.pi/0.5)
    traj.add_fragment(CircleTrajectory(radius=3, omega=0.5, angle_offset=np.pi), duration=np.pi/0.5)
    traj.add_fragment(StraightTrajectory(speed=1, direc=[0, 1]), duration=5)

    # --- controllers ---
    controllers = [
        SamsonController(),
        KanayamaController(),
        M2parController(),
        GeometricController(1.0, 1.0, M_FREE.adapt_input, M_FREE.adapt_output),
    ]

    # =======================================================

    traj_prints = {}
    for con in controllers:
        x = deepcopy(x_ini)
        t = deepcopy(t_ini)
        model = M_FREE if isinstance(con, GeometricController) else con.model()
        if not model.is_std:
            x.theta += np.pi / 2

        sim = Simulator(model, con, traj, dt)

        history = []

        while t < T:
            history.append(np.array(astuple(x)))
            sim.step(x, t)
            t += dt

        history = np.array(history)
        # Realigner avec le vecteur vitesse si pas standard
        if not model.is_std:
            history[:, 2] -= np.pi / 2
        traj_prints[str(con)] = history

    print("Simulation finished")
    animate(traj_prints, traj, dt)


if __name__ == "__main__":
    main()
