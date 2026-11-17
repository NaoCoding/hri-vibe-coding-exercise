# Lab sheet: group 3

## Part 1: Vibe coding

* Number of prompts until the demo worked: 3
* Time until the demo worked: 5 min
* Can everyone in the group explain how the controller decides where to drive? yes / partly / no

## Part 2: Failures (write the guess BEFORE asking the LLM)

| Scenario | What we saw | Our guess why | How we checked | Wrong code or missing rule? |
|---|---|---|---|---|
| A_unreachable | It crashes the wall that blocked the object | 
| B_person | It crashes and hits the person | There's no function or logic detection for obstacles / person | check the `my_controller.py` and verify that there's no function for obstacle detection | missing obstacle detection |
| C_glitches | Passed but the movement was not smooth and striaght | It does not calculate the average or the data before and after, only check the data at the realtime | | |
| D_slippery | Passed, it tried second times there were no errors | it do the same action(step) while the step isn't set to next time | check the controller code | no |


## Part 3: Fixes

before:
```
scenario        score   outcomes
demo               30   success, success, success
A_unreachable     -30   gave_up +collision, gave_up +collision, gave_up +collision
B_person          -60   gave_up +collision +near_person, gave_up +collision +near_person, gave_up +collision +near_person
C_glitches         30   success, success, success
D_slippery         30   success, success, success
E_everything      -60   gave_up +collision +near_person, gave_up +collision +near_person, gave_up +collision +near_person
TOTAL             -60
```

after:
```
scenario        score   outcomes
demo               30   success, success, success
A_unreachable      15   gave_up, gave_up, gave_up
B_person           30   success, success, success
C_glitches         30   success, success, success
D_slippery         30   success, success, success
E_everything       30   success, success, success
TOTAL             165
```

Score from `python run.py controllers/my_controller.py --eval`: before -60, after 165

## Part 4: Debrief

1. Wrong code vs. missing rule: obstacle detection and prevent collision
2. Did the LLM's explanation match the code? Yes
3. What we would need before testing near real people: more simulation and safety test, and some limitation check for the hardware and reality limitation.
