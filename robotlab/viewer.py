"""pygame window. Keys: SPACE pause, R restart, S next seed, +/- speed, Q quit."""
import math

import pygame

from .sim import HEIGHT, PERSON_RADIUS, PERSON_RULE_DISTANCE, ROBOT_RADIUS, WIDTH, Simulation, score

PX, M, PANEL = 100, 20, 300  # pixels per metre, margin, side panel width


def show(sim: Simulation):
    pygame.init()
    screen = pygame.display.set_mode((int(WIDTH * PX) + 2 * M + PANEL, int(HEIGHT * PX) + 2 * M))
    pygame.display.set_caption("HRI fetch lab")
    font = pygame.font.SysFont("dejavusansmono,consolas,monospace", 15)
    clock = pygame.time.Clock()
    speed, paused, acc = 1.0, False, 0.0
    controller_cls = type(sim.controller)

    def p(x, y):
        return int(M + x * PX), int(M + (HEIGHT - y) * PX)

    while True:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT or (ev.type == pygame.KEYDOWN and ev.key in (pygame.K_q, pygame.K_ESCAPE)):
                pygame.quit()
                return sim.result
            if ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_SPACE:
                    paused = not paused
                elif ev.key in (pygame.K_r, pygame.K_s):
                    seed = sim.seed + (ev.key == pygame.K_s)
                    sim = Simulation(sim.scenario, controller_cls(), seed)
                elif ev.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                    speed = min(16, speed * 2)
                elif ev.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                    speed = max(0.25, speed / 2)
        acc += 0 if paused else clock.get_time() / 1000 * speed
        while acc >= sim.task.dt and not sim.done:
            sim.step()
            acc -= sim.task.dt

        screen.fill((255, 255, 255))
        pygame.draw.rect(screen, (245, 244, 240), (M, M, WIDTH * PX, HEIGHT * PX))
        pygame.draw.circle(screen, (150, 210, 160), p(*sim.zone), int(sim.zone_radius * PX))
        for b in sim.boxes:
            x0, y1 = p(b.x0, b.y0)
            x1, y0 = p(b.x1, b.y1)
            pygame.draw.rect(screen, (90, 90, 110), (x0, y0, x1 - x0, y1 - y0))
        pygame.draw.rect(screen, (50, 50, 55), (M, M, WIDTH * PX, HEIGHT * PX), 3)
        if len(sim.result.trail) > 1:
            pygame.draw.lines(screen, (180, 190, 205), False, [p(x, y) for x, y in sim.result.trail], 2)
        for person in sim.people:
            pygame.draw.circle(screen, (200, 200, 215), p(person.x, person.y), int(PERSON_RULE_DISTANCE * PX), 1)
            pygame.draw.circle(screen, (60, 120, 220), p(person.x, person.y), int(PERSON_RADIUS * PX))
        ox, oy = p(*sim.object)
        pygame.draw.rect(screen, (240, 140, 30), (ox - 9, oy - 9, 18, 18))
        front = sim.front_ray()
        reach = ROBOT_RADIUS + (front if front != math.inf else 3.0)
        tip = (sim.x + reach * math.cos(sim.theta), sim.y + reach * math.sin(sim.theta))
        pygame.draw.line(screen, (200, 80, 80), p(sim.x, sim.y), p(*tip), 1)
        pygame.draw.circle(screen, (240, 140, 30) if sim.holding else (70, 90, 110), p(sim.x, sim.y), int(ROBOT_RADIUS * PX))
        nose = (sim.x + ROBOT_RADIUS * math.cos(sim.theta), sim.y + ROBOT_RADIUS * math.sin(sim.theta))
        pygame.draw.line(screen, (255, 255, 255), p(sim.x, sim.y), p(*nose), 3)

        r, cmd = sim.result, sim.last_cmd
        lines = [
            f"{sim.scenario.name}  seed {sim.seed}",
            f"t = {sim.t:5.1f} s   x{speed:g}" + ("  PAUSED" if paused else ""),
            "",
            f"label:   {cmd.label}",
            f"v={cmd.v:+.2f}  omega={cmd.omega:+.2f}",
            f"gripper: {cmd.gripper}  holding={sim.holding}",
            "",
            "what really happened:",
            f"collisions:        {r.collisions}",
            f"moved near person: {r.person_violations} steps",
            f"grasp attempts:    {r.grasp_attempts}",
            f"delivered:         {r.delivered}",
        ]
        if sim.done:
            pts, items = score(r)
            lines += ["", f"END: {r.outcome}", f"score {pts:+d}"] + [f"  {q:+d} {n}" for n, q in items]
            if r.error:
                lines += [r.error.strip().splitlines()[-1][:34]]
        lines += ["", "SPACE pause  R restart", "S next seed  +/- speed", "Q quit"]
        for i, text in enumerate(lines):
            bad = text.startswith(("collisions", "moved")) and not text.endswith((" 0", " 0 steps"))
            color = (200, 40, 40) if bad or text.startswith(("END: timeout", "END: crash", "END: false")) else (30, 30, 35)
            screen.blit(font.render(text, True, color), (int(WIDTH * PX) + 2 * M, M + i * 21))
        pygame.display.flip()
        clock.tick(60)
