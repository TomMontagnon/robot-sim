import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from api.abstract_traj import AbstractTrajectory


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


def animate(history: np.ndarray, traj: AbstractTrajectory, dt: float) -> None:
    """
    history: Nx3 array of robot states
    traj: trajectory object (for reference plot)
    """

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_aspect("equal")
    ax.grid(True)

    # --- Plot desired trajectory (static) ---
    t_ref = np.linspace(0, dt * len(history), 500)
    xd, yd = [], []
    for t in t_ref:
        x, y, _ = traj.evaluate_pos(t)
        xd.append(x)
        yd.append(y)

    ax.plot(xd, yd, "r--", label="Trajectoire désirée")

    # --- Dynamic elements ---
    (traj_real,) = ax.plot([], [], "b-", lw=2, label="Trajectoire réelle")
    robot_patch = plt.Polygon([[0, 0], [0, 0], [0, 0]], color="b", alpha=0.7)

    ax.add_patch(robot_patch)

    # --- Ajouter point idéal rouge ---
    (ideal_point,) = ax.plot([], [], "ro", label="Position idéale instantanée")

    ax.legend()

    # --- Axis limits ---
    margin = 1.0
    ax.set_xlim(min(xd) - margin, max(xd) + margin)
    ax.set_ylim(min(yd) - margin, max(yd) + margin)

    # --- Animation update ---
    def update(frame: int) -> tuple:
        x, y, theta = history[frame]

        traj_real.set_data(history[:frame, 0], history[:frame, 1])

        P = robot_shape(x, y, theta)
        robot_patch.set_xy(P)

        # Position idéale à cet instant
        t = frame * dt
        x_ideal, y_ideal, _ = traj.evaluate_pos(t)
        ideal_point.set_data([x_ideal], [y_ideal])

        return traj_real, robot_patch, ideal_point

    _ani = FuncAnimation(
        fig, update, frames=len(history), interval=dt * 1000, blit=True
    )

    plt.show()
