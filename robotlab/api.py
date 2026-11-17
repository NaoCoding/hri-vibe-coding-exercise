"""The robot API: everything your controller can see and do.

Write a class with two methods:

    class MyController(Controller):
        def reset(self, task):   # once per run
            ...
        def step(self, obs):     # every 0.05 s, must return a Command
            return Command(v=0.3, omega=0.0)

World frame: metres, x to the right, y up, angles in radians
counter-clockwise from the x axis. The pose is always exact.
"""
import math
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass(frozen=True)
class TaskInfo:
    object_position: Tuple[float, float]  # where to pick up the object
    zone_position: Tuple[float, float]  # centre of the delivery zone
    zone_radius: float  # release the object inside this circle
    max_speed: float = 0.5  # |v| limit [m/s]
    max_turn_rate: float = 2.0  # |omega| limit [rad/s]
    grasp_range: float = 0.4  # grasping works if the robot centre is this close to the object [m]
    dt: float = 0.05  # control period [s]


@dataclass(frozen=True)
class Observation:
    t: float  # time since start [s]
    x: float  # robot position [m]
    y: float
    theta: float  # robot heading [rad]
    front_distance: Optional[float]  # free space in front of the robot's bumper [m];
    #                                  math.inf = nothing within 3 m, None = sensor glitch
    human_distance: Optional[float]  # robot centre to nearest person's centre [m];
    #                                  math.inf = nobody around, None = sensor glitch
    holding: bool  # True if the object is in the gripper


@dataclass
class Command:
    v: float = 0.0  # forward speed [m/s]
    omega: float = 0.0  # turn rate [rad/s], positive = turn left
    gripper: Optional[str] = None  # None, "grasp" or "release"
    status: str = "running"  # "running", "success" (I delivered it) or "failure" (I give up)
    label: str = ""  # any text, shown in the viewer (use it for your state name)


class Controller:
    def reset(self, task: TaskInfo) -> None:
        self.task = task

    def step(self, obs: Observation) -> Command:
        raise NotImplementedError


# -- helpers for writing tests -------------------------------------------------
def make_task(**changes) -> TaskInfo:
    """A TaskInfo like the one in the demo world."""
    return TaskInfo(**{"object_position": (6.5, 4.5), "zone_position": (1.5, 4.5), "zone_radius": 0.5, **changes})


def make_obs(**changes) -> Observation:
    """An Observation with sensible defaults; override any field, e.g. make_obs(human_distance=0.5)."""
    defaults = dict(t=0.0, x=1.0, y=1.0, theta=0.0, front_distance=math.inf,
                    human_distance=math.inf, holding=False)
    return Observation(**{**defaults, **changes})
