"""Tests for your controller. Run with:  python -m pytest

Two tests are written for you. Write the three marked TODO (remove the
pytest.skip line). Each test should fail for your Phase-1 controller and pass
after your fix.
"""
import pytest

from controllers.my_controller import MyController
from robotlab import make_obs, make_task, run_episode


def new_controller():
    c = MyController()
    c.reset(make_task())
    return c


def test_demo_is_delivered_without_collision():
    for seed in range(3):
        result = run_episode(MyController(), "demo", seed=seed)
        assert result.outcome == "success", result.summary()
        assert result.collisions == 0, result.summary()


def test_robot_stands_still_when_a_person_is_close():
    c = new_controller()
    cmd = c.step(make_obs(human_distance=0.6))
    assert cmd.v == 0.0


def test_unreachable_object_ends_with_failure():
    pytest.skip("TODO: run 'A_unreachable'; the outcome should be 'gave_up', not 'timeout'")


def test_sensor_glitch_does_not_crash_and_does_not_drive():
    pytest.skip("TODO: step with make_obs(front_distance=None) and make_obs(human_distance=None)")


def test_no_success_claim_with_empty_gripper():
    pytest.skip("TODO: run 'D_slippery'; the outcome must never be 'false_success'")
