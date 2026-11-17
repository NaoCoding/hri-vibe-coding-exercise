"""Run a controller.

    python run.py                                           # your controller in the demo world, with window
    python run.py controllers/my_controller.py --scenario B_person --seed 2
    python run.py controllers/my_controller.py --headless   # no window, just the result
    python run.py controllers/my_controller.py --eval       # all scenarios x 3 seeds, with score
"""
import argparse
import importlib.util
import inspect
import os

from robotlab import SCENARIOS, Controller, Simulation, score


def load_controller_class(path):
    spec = importlib.util.spec_from_file_location("student_controller", os.path.abspath(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    classes = [c for _, c in inspect.getmembers(module, inspect.isclass)
               if issubclass(c, Controller) and c is not Controller and c.__module__ == module.__name__]
    if len(classes) != 1:
        raise SystemExit(f"{path} must define exactly one Controller class, found {[c.__name__ for c in classes]}")
    return classes[0]


def evaluate(cls, seeds):
    total = 0
    print(f"{'scenario':<15}{'score':>6}   outcomes")
    for name, scenario in SCENARIOS.items():
        results = [Simulation(scenario, cls(), seed).run() for seed in seeds]
        points = [score(r)[0] for r in results]
        total += sum(points)
        print(f"{name:<15}{sum(points):>6}   " + ", ".join(
            r.outcome + (" +collision" if r.collisions else "") + (" +near_person" if r.person_violations else "")
            for r in results))
    print(f"{'TOTAL':<15}{total:>6}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("controller", nargs="?", default="controllers/my_controller.py")
    ap.add_argument("--scenario", default="demo", choices=list(SCENARIOS))
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--eval", action="store_true", help="score all scenarios")
    ap.add_argument("--seeds", type=int, default=3, help="with --eval: number of seeds")
    ap.add_argument("--first-seed", type=int, default=0, help="with --eval: first seed")
    a = ap.parse_args()

    cls = load_controller_class(a.controller)
    if a.eval:
        evaluate(cls, range(a.first_seed, a.first_seed + a.seeds))
        return
    sim = Simulation(a.scenario, cls(), a.seed)
    if a.headless:
        result = sim.run()
    else:
        from robotlab.viewer import show
        result = show(sim)
    print(result.summary())
    print("score: %+d %s" % score(result))


if __name__ == "__main__":
    main()
