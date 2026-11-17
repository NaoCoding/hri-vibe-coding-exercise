# Vibe-code a robot, then make it trustworthy

A robot must **drive to an object, grasp it, drive to the green delivery zone
and release it**. You get a ready-made simulator. You write the controller:
first fast with an LLM, then carefully.

## Setup

```bash
python -m venv .venv
```
```bash
source .venv/bin/activate        # Windows: .venv\Scripts\activate
```
```bash
pip install -r requirements.txt
```
```bash
python -m pytest tests/test_simulator.py     # should say "3 passed"
```

## Commands

| What | Command |
|---|---|
| Watch your robot | `python run.py controllers/my_controller.py` |
| Another scenario / seed | `python run.py controllers/my_controller.py --scenario B_person --seed 2` |
| Without window | `python run.py controllers/my_controller.py --headless` |
| Your score on all scenarios | `python run.py controllers/my_controller.py --eval` |
| Your tests | `python -m pytest` |

In the window: **SPACE** pause, **R** restart, **S** next seed, **+/−** speed, **Q** quit.
The panel shows *what really happened* (collisions, moving near a person,
delivered or not), which your controller cannot see.

## The robot API ([robotlab/api.py](robotlab/api.py))

```python
from robotlab import Command, Controller

class MyController(Controller):
    def reset(self, task):       # once per run
        self.task = task
    def step(self, obs):         # 20 times per second
        return Command(v=0.3, omega=0.0, label="DRIVE")
```

| `task` | |
|---|---|
| `object_position`, `zone_position`, `zone_radius` | where things are (x, y in metres) |
| `max_speed` 0.5 m/s, `max_turn_rate` 2 rad/s | limits |
| `grasp_range` 0.4 m | grasping works if the robot is this close to the object |

| `obs` | |
|---|---|
| `t` | time in seconds |
| `x`, `y`, `theta` | robot pose (exact), theta in radians, 0 = facing right |
| `front_distance` | free space in front of the robot in metres, `math.inf` if nothing within 3 m, **`None` on a sensor glitch** |
| `human_distance` | distance to the nearest person (centre to centre), `math.inf` if nobody is around, **`None` on a sensor glitch** |
| `holding` | `True` if the object is in the gripper |

| `Command(...)` | |
|---|---|
| `v`, `omega` | forward speed (m/s) and turn rate (rad/s, positive = left) |
| `gripper` | `None`, `"grasp"` or `"release"` |
| `status` | `"running"`, `"success"` (ends the run: "I delivered it") or `"failure"` (ends the run: "I can't do it") |
| `label` | text shown in the window, e.g. your current state |

**Rules for a good robot**

1. Never collide with walls, boxes or people.
2. Whenever a person is closer than **1.0 m**, stand still (turning on the spot is fine).
3. Only report `"success"` if the object really is in the zone.
4. Stop within **60 seconds**: report `"success"` or `"failure"`, never just keep going.

## Scenarios

| Scenario | What is different |
|---|---|
| `demo` | nothing, an empty room |
| `A_unreachable` | the object is locked in a box |
| `B_person` | a person steps into the robot's path |
| `C_glitches` | 20 % of the sensor readings are `None` |
| `D_slippery` | the first grasp attempt fails |
| `E_everything` | B, C and D together |

## Assignment

Work in groups: One person types the prompts and keeps the
[lab sheet](docs/LAB_SHEET.md); swap after each part.

### Part 1: Vibe coding

Give your LLM the file `robotlab/api.py` and this prompt:

> Write a Python controller for a simulated mobile robot. It should drive to
> the object, grasp it, drive to the delivery zone and release it. Use the
> provided API (the `Controller` class with `reset` and `step`). Keep it
> simple and make it work in the demo world. Explain how it works.

Save the answer as `controllers/my_controller.py` and run it. Goal: the robot
delivers the object in `demo`. Count your prompts. When it works, **copy the
file to `controllers/phase1_controller.py`**; you will need it again.

### Part 2: Break it

**Do not use an LLM for this part!** 

Run your controller in `B_person` and in one more scenario that your
instructor assigns (A, C or D). For each failure:

1. Describe what you see.
2. **Write down why you think it happens.** Try to find the line in the code that fails or explain what could be missing.
3. Check your guess: pause, restart, read the error message, add a `print`.

### Part 3: Fix it and prove it

**Now you may use the LLM again.**
Remember to keep the [lab sheet](docs/LAB_SHEET.md)!

* Fix the failures you found (manually or using an LLM). Use **states**: `Command(label=...)`, for example
  `TO_OBJECT → GRASP → TO_ZONE → RELEASE → DONE`, plus `FAILED`.
Example:
```python
if self.state == "TO_OBJECT":
  ...
  return Command(..., label=self.state, ...)
  ...
elif self.state == "GRASP":
  ...
```

  For every state, decide when you leave it and what happens if it takes too long.
* Write the three TODO tests in [tests/test_my_controller.py](tests/test_my_controller.py).
  Each one should **fail for `phase1_controller.py`** and pass for your new controller.
  To check, temporarily change the import at the top of the test file.
* Run `python run.py controllers/my_controller.py --eval`.

### Part 4: Final check and debrief

Your instructor runs your controller on new seeds and hidden tests. Points per run:

| | Points |
|---|---|
| Object delivered | +10 |
| Gave up cleanly on the unreachable object | +5 |
| Collision | −10 |
| Moved while a person was closer than 1 m | −10 |
| Timeout or crash | −5 |
| Claimed success without delivering | −5 |

Then discuss:

1. Which bugs were wrong code, and which were rules nobody had written down?
2. Did the LLM's explanation match what the code actually did?
3. What would you need to see before letting this robot drive near real people?
