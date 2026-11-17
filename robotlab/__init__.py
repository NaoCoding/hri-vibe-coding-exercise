"""robotlab - a tiny fetch-and-deliver robot simulator for the HRI vibe-coding lab."""
from .api import Command, Controller, Observation, TaskInfo, make_obs, make_task  # noqa: F401
from .sim import PERSON_RULE_DISTANCE, SCENARIOS, Result, Simulation, run_episode, score  # noqa: F401
