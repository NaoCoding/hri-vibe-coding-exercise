"""Simple point-to-point fetch controller for the robotlab demo world."""
import math

from robotlab import Command, Controller


class MyController(Controller):
    def reset(self, task):
        self.task = task
        self.state = "TO_OBJECT"

    def _drive_to(self, obs, target, stop_distance):
        dx = target[0] - obs.x
        dy = target[1] - obs.y
        distance = math.hypot(dx, dy)
        if distance <= stop_distance:
            return None

        heading = math.atan2(dy, dx)
        error = math.atan2(math.sin(heading - obs.theta), math.cos(heading - obs.theta))
        omega = max(-self.task.max_turn_rate, min(self.task.max_turn_rate, 2.0 * error))

        # Turn before driving when facing well away from the target.
        if abs(error) > 0.45:
            return Command(v=0.0, omega=omega, label=self.state)

        speed = min(self.task.max_speed, 0.8 * distance)
        return Command(v=speed, omega=omega, label=self.state)

    def step(self, obs):
        if obs.t >= 59.0:
            return Command(status="failure", label="FAILED")

        if self.state == "TO_OBJECT":
            action = self._drive_to(obs, self.task.object_position, self.task.grasp_range * 0.75)
            if action is None:
                self.state = "GRASP"
            else:
                return action

        if self.state == "GRASP":
            if obs.holding:
                self.state = "TO_ZONE"
            else:
                return Command(gripper="grasp", label=self.state)

        if self.state == "TO_ZONE":
            action = self._drive_to(obs, self.task.zone_position, self.task.zone_radius * 0.5)
            if action is None:
                self.state = "RELEASE"
            else:
                return action

        if self.state == "RELEASE":
            if obs.holding:
                return Command(gripper="release", status="success", label=self.state)
            self.state = "DONE"

        return Command(status="success", label=self.state)
