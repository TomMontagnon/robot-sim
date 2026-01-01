from api.abstract_traj import AbstractTrajectory
from dataclasses import dataclass


@dataclass
class TrajectoryFragment:
    traj: AbstractTrajectory
    duration: float
    x0: float = 0
    y0: float = 0
    theta0: float = 0


class CompositeTrajectory(AbstractTrajectory):
    def __init__(self) -> None:
        self.fragments = []
        self.total_duration = 0.0

    def add_fragment(self, traj: AbstractTrajectory, duration: float) -> None:
        if not self.fragments:
            x0 = y0 = theta0 = 0.0
        else:
            prev = self.fragments[-1]
            xf, yf, thetaf, *_ = prev.traj.evaluate(prev.duration)
            x0 = prev.x0 + xf
            y0 = prev.y0 + yf
            theta0 = thetaf

        frag = TrajectoryFragment(traj, duration, x0, y0, theta0)

        self.fragments.append(frag)
        self.total_duration += duration

    def evaluate(self, t: float) -> tuple:
        # Trouver le fragment actif
        acc = 0.0
        for frag in self.fragments:
            if acc <= t < acc + frag.duration:
                local_t = t - acc
                x, y, theta, vx, vy = frag.traj.evaluate(local_t)
                # Replacer dans le repère global
                return (x + frag.x0, y + frag.y0, theta, vx, vy)
            acc += frag.duration

        # Si t dépasse la trajectoire => rester au dernier point
        last = self.fragments[-1]
        x, y, theta, vx, vy = last.traj.evaluate(last.duration)
        return (x + last.x0, y + last.y0, theta, vx, vy)
