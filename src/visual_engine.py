import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from api.abstract_traj import AbstractTrajectory
import itertools


def robot_shape(x: float, y: float, theta: float, L: float = 0.3) -> np.ndarray:
    """
    Returns triangle vertices in world frame
    """
    p1 = np.array([L, 0])
    p2 = np.array([-L, L / 2])
    p3 = np.array([-L, -L / 2])

    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])

    P = np.vstack((p1, p2, p3)) @ R.T
    P[:, 0] += x
    P[:, 1] += y

    return P


def animate(history_dico: dict, traj: AbstractTrajectory, dt: float) -> None:
    """
    history_dico: {nom_robot: Nx3 numpy array}
    """

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_aspect("equal")
    ax.grid(True)

    # --- Plot desired trajectory (static) ---
    t_ref = np.linspace(0, dt * max(len(h) for h in history_dico.values()), 500)
    xd, yd = [], []
    for t in t_ref:
        x, y, _, _, _ = traj.evaluate(t)
        xd.append(x)
        yd.append(y)

    ax.plot(xd, yd, "r--", label="Trajectoire désirée")

    # --- Point idéal global (rouge) ---
    (ideal_point,) = ax.plot(
        [], [], "o", color="red", markersize=6, label="Point idéal"
    )
    # =======================================================================================
    #           Création des éléments graphiques pour CHAQUE robot
    # =======================================================================================

    couleurs = itertools.cycle(plt.cm.tab20.colors)

    robots = {}  # stockage des éléments graphiques

    for name, history in history_dico.items():
        color = next(couleurs)

        # courbe réelle
        (traj_real,) = ax.plot([], [], "-", lw=2, color=color, label=f"{name}")

        # robot patch
        robot_patch = plt.Polygon([[0, 0], [0, 0], [0, 0]], color=color, alpha=0.6)
        ax.add_patch(robot_patch)

        robots[name] = {
            "history": history,
            "traj_real": traj_real,
            "robot_patch": robot_patch,
            "color": color,
        }

    ax.legend()

    # --- Axis limits ---
    margin = 1.0
    ax.set_xlim(min(xd) - margin, max(xd) + margin)
    ax.set_ylim(min(yd) - margin, max(yd) + margin)

    # =======================================================================================
    #                                FONCTION UPDATE
    # =======================================================================================

    max_frames = max(len(v["history"]) for v in robots.values())

    def update(frame: int):
        artists = []

        for name, data in robots.items():
            history = data["history"]

            # éviter dépassement si history plus court
            if frame < len(history):
                x, y, theta = history[frame]

                # tracer la trajectoire réelle
                data["traj_real"].set_data(history[:frame, 0], history[:frame, 1])
                artists.append(data["traj_real"])

                # robot patch
                P = robot_shape(x, y, theta)
                data["robot_patch"].set_xy(P)
                artists.append(data["robot_patch"])

                # point idéal
                t = frame * dt
                x_ideal, y_ideal, _, _, _ = traj.evaluate(t)
                ideal_point.set_data([x_ideal], [y_ideal])
                artists.append(ideal_point)

        return artists

    # --- Animation ---
    _ani = FuncAnimation(fig, update, frames=max_frames, interval=dt * 1000, blit=True)

    plt.legend(loc="lower center", bbox_to_anchor=(0.5, 1.05))
    plt.tight_layout()
    plt.show()
