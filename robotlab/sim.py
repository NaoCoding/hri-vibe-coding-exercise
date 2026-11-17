"""The simulator. You do not need to read this file to do the lab.

    from robotlab import run_episode
    result = run_episode(MyController(), "B_person", seed=0)
    print(result.summary())
"""
import math
import random
import traceback
from dataclasses import dataclass, field, replace
from typing import List, Optional

from .api import Command, Observation, TaskInfo

WIDTH, HEIGHT = 8.0, 6.0
ROBOT_RADIUS = 0.2
PERSON_RADIUS = 0.25
SENSOR_RANGE = 3.0
PERSON_RULE_DISTANCE = 1.0  # closer than this (centre to centre) the robot must stand still
STANDSTILL_SPEED = 0.05


@dataclass(frozen=True)
class Scenario:
    name: str = "custom"
    unreachable: bool = False  # Failure A: the object is locked in a box
    person: bool = False  # Failure B: a person steps into the robot's path
    glitch_prob: float = 0.0  # Failure C: chance that a sensor reading is None
    slippery: bool = False  # Failure D: the first grasp attempt fails
    time_limit: float = 60.0


SCENARIOS = {s.name: s for s in [
    Scenario("demo"),
    Scenario("A_unreachable", unreachable=True),
    Scenario("B_person", person=True),
    Scenario("C_glitches", glitch_prob=0.2),
    Scenario("D_slippery", slippery=True),
    Scenario("E_everything", person=True, glitch_prob=0.2, slippery=True),
]}


@dataclass
class Result:
    scenario: Scenario
    seed: int
    outcome: str = "running"  # success | false_success | gave_up | timeout | crash
    delivered: bool = False
    collisions: int = 0
    person_violations: int = 0  # steps in which the robot moved while a person was closer than 1 m
    limit_violations: int = 0  # commands faster than the limits (they are clipped)
    grasp_attempts: int = 0
    time: float = 0.0
    error: str = ""
    trail: List[tuple] = field(default_factory=list)

    def summary(self) -> str:
        s = (f"{self.scenario.name} seed={self.seed}: {self.outcome} after {self.time:.1f}s | "
             f"delivered={self.delivered} collisions={self.collisions} "
             f"moved_near_person={self.person_violations} steps grasp_attempts={self.grasp_attempts}")
        if self.error:
            s += "\n" + self.error.strip()
        return s


def score(r: Result):
    """Points for one run, as (total, [(reason, points), ...])."""
    items = []
    if r.outcome == "success":
        items.append(("delivered", +10))
    if r.scenario.unreachable and r.outcome == "gave_up" and not r.collisions:
        items.append(("gave up cleanly on unreachable object", +5))
    if r.collisions:
        items.append(("collision", -10))
    if r.person_violations:
        items.append(("moved near a person", -10))
    if r.outcome in ("timeout", "crash"):
        items.append((r.outcome, -5))
    if r.outcome == "false_success":
        items.append(("claimed success without delivering", -5))
    return sum(p for _, p in items), items


# -- world -----------------------------------------------------------------------
@dataclass
class Box:
    x0: float
    y0: float
    x1: float
    y1: float

    def distance_to(self, x, y):
        dx = max(self.x0 - x, 0.0, x - self.x1)
        dy = max(self.y0 - y, 0.0, y - self.y1)
        return math.hypot(dx, dy)

    def ray(self, x, y, dx, dy):
        """Distance along a unit ray to this box, or None."""
        lo, hi = 0.0, math.inf
        for o, d, a, b in ((x, dx, self.x0, self.x1), (y, dy, self.y0, self.y1)):
            if abs(d) < 1e-12:
                if not a <= o <= b:
                    return None
                continue
            t1, t2 = sorted(((a - o) / d, (b - o) / d))
            lo, hi = max(lo, t1), min(hi, t2)
        return lo if lo <= hi else None


def ray_to_circle(x, y, dx, dy, cx, cy, r):
    fx, fy = x - cx, y - cy
    b = fx * dx + fy * dy
    disc = b * b - (fx * fx + fy * fy - r * r)
    if disc < 0:
        return None
    t = -b - math.sqrt(disc)
    return t if t >= 0 else None


class Person:
    """Walks to a spot on the robot's path, stands there, then walks back."""

    def __init__(self, start, spot, wait=5.0, speed=0.8):
        self.x, self.y = start
        self.route = [spot, start]
        self.spot = spot
        self.wait, self.speed = wait, speed
        self.started = False
        self.wait_until = None

    def update(self, t, dt, robot_xy):
        if not self.started:
            self.started = math.dist(robot_xy, self.spot) < 2.6
            return
        if not self.route:
            return
        if self.wait_until is not None and t < self.wait_until:
            return
        gx, gy = self.route[0]
        d = math.hypot(gx - self.x, gy - self.y)
        step = min(d, self.speed * dt)
        nx = self.x + (gx - self.x) / d * step if d > 0 else gx
        ny = self.y + (gy - self.y) / d * step if d > 0 else gy
        new_gap = math.dist((nx, ny), robot_xy)
        if new_gap < ROBOT_RADIUS + PERSON_RADIUS + 0.05 and new_gap < math.dist((self.x, self.y), robot_xy):
            return  # people do not walk into the robot (but can always step away)
        self.x, self.y = nx, ny
        if step == d:
            self.route.pop(0)
            self.wait_until = t + self.wait if len(self.route) == 1 else None


class Simulation:
    def __init__(self, scenario, controller, seed=0, **changes):
        if isinstance(scenario, str):
            scenario = SCENARIOS[scenario]
        self.scenario = replace(scenario, **changes) if changes else scenario
        self.controller = controller
        self.seed = seed
        rng = random.Random(seed)
        self.noise = random.Random(seed + 1000)

        self.x, self.y, self.theta = 1.0 + rng.uniform(-0.2, 0.2), 1.0 + rng.uniform(-0.2, 0.2), 0.0
        self.object = (6.5 + rng.uniform(-0.3, 0.3), 4.5 + rng.uniform(-0.3, 0.3))
        self.zone, self.zone_radius = (1.5, 4.5), 0.5
        self.holding = False
        self.slips_left = 1 if self.scenario.slippery else 0
        self.walls = [Box(-1, -1, WIDTH + 1, 0), Box(-1, HEIGHT, WIDTH + 1, HEIGHT + 1),
                      Box(-1, 0, 0, HEIGHT), Box(WIDTH, 0, WIDTH + 1, HEIGHT)]
        self.boxes = []
        if self.scenario.unreachable:
            ox, oy = self.object
            a, w = 0.6, 0.1
            self.boxes = [Box(ox - a, oy - a, ox + a, oy - a + w), Box(ox - a, oy + a - w, ox + a, oy + a),
                          Box(ox - a, oy - a, ox - a + w, oy + a), Box(ox + a - w, oy - a, ox + a, oy + a)]
        self.people = []
        if self.scenario.person:
            dx, dy = self.object[0] - self.x, self.object[1] - self.y
            n = math.hypot(dx, dy)
            spot = (self.x + 3.0 * dx / n, self.y + 3.0 * dy / n)  # 3 m along the direct path
            start = (spot[0] + 2.0 * dy / n, spot[1] - 2.0 * dx / n)
            self.people = [Person(start, spot, wait=4.0 + rng.uniform(0, 2))]

        self.t = 0.0
        self.done = False
        self.in_contact = False
        self.last_cmd = Command()
        self.result = Result(self.scenario, seed)
        self.task = TaskInfo(self.object, self.zone, self.zone_radius)
        try:
            controller.reset(self.task)
        except Exception:
            self._finish("crash", traceback.format_exc())

    # -- sensing -----------------------------------------------------------------
    def person_distance(self):
        return min((math.hypot(p.x - self.x, p.y - self.y) for p in self.people), default=math.inf)

    def front_ray(self):
        dx, dy = math.cos(self.theta), math.sin(self.theta)
        hits = [b.ray(self.x, self.y, dx, dy) for b in self.walls + self.boxes]
        hits += [ray_to_circle(self.x, self.y, dx, dy, p.x, p.y, PERSON_RADIUS) for p in self.people]
        d = min((h for h in hits if h is not None), default=math.inf) - ROBOT_RADIUS
        return max(0.0, d) if d <= SENSOR_RANGE else math.inf

    def observe(self):
        glitch = lambda value: None if self.noise.random() < self.scenario.glitch_prob else value  # noqa: E731
        return Observation(t=round(self.t, 3), x=self.x, y=self.y, theta=self.theta,
                           front_distance=glitch(self.front_ray()),
                           human_distance=glitch(self.person_distance()), holding=self.holding)

    # -- one step ----------------------------------------------------------------
    def blocked(self, x, y):
        if any(b.distance_to(x, y) < ROBOT_RADIUS for b in self.walls + self.boxes):
            return True
        return any(math.hypot(p.x - x, p.y - y) < ROBOT_RADIUS + PERSON_RADIUS for p in self.people)

    def step(self):
        if self.done:
            return False
        res, task = self.result, self.task
        try:
            cmd = self.controller.step(self.observe())
            if not isinstance(cmd, Command):
                raise TypeError(f"step() must return a Command, not {type(cmd).__name__}")
            v, omega = float(cmd.v), float(cmd.omega)
        except Exception:
            self._finish("crash", traceback.format_exc())
            return False
        self.last_cmd = cmd

        if abs(v) > task.max_speed + 1e-9 or abs(omega) > task.max_turn_rate + 1e-9:
            res.limit_violations += 1
        v = max(-task.max_speed, min(task.max_speed, v))
        omega = max(-task.max_turn_rate, min(task.max_turn_rate, omega))
        if self.person_distance() < PERSON_RULE_DISTANCE and abs(v) > STANDSTILL_SPEED:
            res.person_violations += 1

        self.theta = math.atan2(math.sin(self.theta + omega * task.dt), math.cos(self.theta + omega * task.dt))
        nx, ny = self.x + v * math.cos(self.theta) * task.dt, self.y + v * math.sin(self.theta) * task.dt
        if abs(v) > 1e-9 and self.blocked(nx, ny):
            if not self.in_contact:
                res.collisions += 1
            self.in_contact = True
        else:
            self.in_contact = False
            self.x, self.y = nx, ny

        if cmd.gripper == "grasp" and not self.holding:
            res.grasp_attempts += 1
            if math.dist((self.x, self.y), self.object) <= task.grasp_range and not res.delivered:
                if self.slips_left:
                    self.slips_left -= 1  # the object slips out of the gripper
                else:
                    self.holding = True
        elif cmd.gripper == "release" and self.holding:
            self.holding = False
            self.object = (self.x, self.y)
            res.delivered = math.dist(self.object, self.zone) <= self.zone_radius
        if self.holding:
            self.object = (self.x, self.y)

        for p in self.people:
            p.update(self.t, task.dt, (self.x, self.y))
        self.t += task.dt
        res.time = self.t
        res.trail.append((self.x, self.y))

        if cmd.status == "success":
            self._finish("success" if res.delivered else "false_success")
        elif cmd.status == "failure":
            self._finish("gave_up")
        elif self.t >= self.scenario.time_limit:
            self._finish("timeout")
        return not self.done

    def _finish(self, outcome, error=""):
        self.done = True
        self.result.outcome, self.result.error = outcome, error

    def run(self):
        while self.step():
            pass
        return self.result


def run_episode(controller, scenario="demo", seed=0, **changes) -> Result:
    """Run one complete episode without a window and return the Result."""
    return Simulation(scenario, controller, seed, **changes).run()
