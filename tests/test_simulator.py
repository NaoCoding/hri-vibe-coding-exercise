"""Checks that the simulator works. Run once after installing:  python -m pytest tests/test_simulator.py"""
from robotlab import Command, Controller, run_episode


class DriveStraight(Controller):
    def step(self, obs):
        return Command(v=0.5)


class LieAboutSuccess(Controller):
    def step(self, obs):
        return Command(status="success")


def test_hitting_the_wall_is_a_collision():
    result = run_episode(DriveStraight(), "demo", time_limit=20)
    assert result.collisions >= 1
    assert result.outcome == "timeout"


def test_success_without_delivery_is_detected():
    assert run_episode(LieAboutSuccess(), "demo").outcome == "false_success"


def test_runs_are_repeatable():
    a = run_episode(DriveStraight(), "C_glitches", seed=4, time_limit=5)
    b = run_episode(DriveStraight(), "C_glitches", seed=4, time_limit=5)
    assert a.trail == b.trail
