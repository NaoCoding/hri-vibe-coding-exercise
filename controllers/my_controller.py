"""Cautious point-to-point fetch controller for the robotlab demo world."""
import math

from robotlab import Command, Controller


class MyController(Controller):
    # The front sensor reports free space from the bumper, not from the centre.
    OBSTACLE_DISTANCE = 0.32
    CLEAR_DISTANCE = 0.65

    def reset(self, task):
        self.task = task
        self.state = "TO_OBJECT"
        self.avoiding = False
        self.avoid_direction = 1.0
        self.avoid_started = 0.0
        self.best_distance = math.inf
        self.last_progress_time = 0.0

    @staticmethod
    def _angle_error(heading, theta):
        return math.atan2(math.sin(heading - theta), math.cos(heading - theta))

    def _navigate(self, obs, target, stop_distance):
        dx, dy = target[0] - obs.x, target[1] - obs.y
        distance = math.hypot(dx, dy)
        if distance <= stop_distance:
            self.avoiding = False
            return None

        heading = math.atan2(dy, dx)
        error = self._angle_error(heading, obs.theta)

        # A missing range reading is not evidence that the path is clear.
        if obs.front_distance is None:
            return Command(v=0.0, omega=0.0, label="SENSOR_WAIT")

        # Apply the human rule even while an avoidance maneuver is active.
        if obs.human_distance is None or obs.human_distance < 1.0:
            return Command(v=0.0, omega=0.0, label="WAIT_PERSON")

        if self.avoiding:
            if obs.front_distance >= self.CLEAR_DISTANCE and obs.t - self.avoid_started >= 0.6:
                self.avoiding = False
            else:
                # Follow a short arc around the obstacle. Keep this state until
                # the sensor sees room to proceed, so steering does not point
                # straight back into the obstacle on the next step.
                speed = 0.20 if obs.front_distance > self.OBSTACLE_DISTANCE else 0.0
                return Command(v=speed, omega=self.avoid_direction * 0.9,
                               label="AVOID_OBSTACLE")

        if obs.front_distance < self.OBSTACLE_DISTANCE:
            self.avoiding = True
            self.avoid_started = obs.t
            self.avoid_direction = 1.0 if error >= 0.0 else -1.0
            return Command(v=0.0, omega=self.avoid_direction * self.task.max_turn_rate,
                           label="AVOID_OBSTACLE")

        omega = max(-self.task.max_turn_rate,
                    min(self.task.max_turn_rate, 2.0 * error))
        if abs(error) > 0.45:
            return Command(v=0.0, omega=omega, label=self.state)

        speed = min(self.task.max_speed, 0.8 * distance)
        # Slow down as sensed free space gets short; the extra margin leaves
        # room for the next control period before the bumper reaches an obstacle.
        if math.isfinite(obs.front_distance):
            speed = min(speed, max(0.0, (obs.front_distance - 0.12) * 1.5))
        return Command(v=speed, omega=omega, label=self.state)

    def _stuck_on_unreachable_object(self, obs, distance):
        # Track actual reduction in distance to the object. A persistent front
        # obstacle near the target means it cannot be safely reached.
        if distance < self.best_distance - 0.06:
            self.best_distance = distance
            self.last_progress_time = obs.t
        blocked = (obs.front_distance is not None and
                   math.isfinite(obs.front_distance) and obs.front_distance < 0.8)
        person_clear = (obs.human_distance is not None and obs.human_distance >= 1.0)
        return (distance < 1.6 and blocked and person_clear and
                obs.t - self.last_progress_time >= 4.0)

    def step(self, obs):
        if obs.t >= 59.0:
            return Command(status="failure", label="FAILED_TIMEOUT")

        if self.state == "TO_OBJECT":
            distance = math.hypot(self.task.object_position[0] - obs.x,
                                  self.task.object_position[1] - obs.y)
            if self._stuck_on_unreachable_object(obs, distance):
                return Command(status="failure", label="FAILED_UNREACHABLE")
            action = self._navigate(obs, self.task.object_position,
                                    self.task.grasp_range * 0.75)
            if action is not None:
                return action
            self.state = "GRASP"

        if self.state == "GRASP":
            if obs.holding:
                self.state = "TO_ZONE"
                self.best_distance = math.inf
                self.last_progress_time = obs.t
            else:
                # Repeated attempts handle a slipped grasp without claiming
                # that delivery succeeded.
                return Command(gripper="grasp", label=self.state)

        if self.state == "TO_ZONE":
            action = self._navigate(obs, self.task.zone_position,
                                    self.task.zone_radius * 0.5)
            if action is not None:
                return action
            self.state = "RELEASE"

        if self.state == "RELEASE":
            if obs.holding:
                return Command(gripper="release", status="success", label=self.state)
            return Command(status="failure", label="FAILED_EMPTY_GRIPPER")

        return Command(status="failure", label="FAILED")
