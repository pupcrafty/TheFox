\
import os
import math
import pygame
import pymunk
import pymunk.pygame_util

from src.forces import ForceManager, ForceSequenceLibrary, SequencePlayer, VALID_TARGETS
from src.stick_figure import StickFigure, StickFigureConfig


W, H = 900, 600


def draw_text(screen, text, pos, size=24):
    font = pygame.font.SysFont("consolas", size, bold=True)
    surf = font.render(text, True, (240, 240, 240))
    screen.blit(surf, pos)


def main():
    pygame.init()
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption("Stick Figure Physics Demo (sensor-only forces)")
    clock = pygame.time.Clock()

    space = pymunk.Space()
    space.gravity = (0, 900)

    # Ground
    ground = pymunk.Segment(space.static_body, (0, H - 50), (W, H - 50), 6)
    ground.friction = 1.0
    space.add(ground)

    cfg = StickFigureConfig()
    fig = StickFigure(space, pos=(W // 2, H // 2 - 40), cfg=cfg)

    fm = ForceManager()
    lib = ForceSequenceLibrary.load(os.path.join("config", "force_sequences.json"))
    player = SequencePlayer(lib, fm)

    draw_options = pymunk.pygame_util.DrawOptions(screen)

    running = True
    while running:
        dt = clock.tick(60) / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                if event.key == pygame.K_SPACE:
                    player.play("demo_spin")
                if event.key == pygame.K_j:
                    player.play("demo_jump")
                if event.key == pygame.K_r:
                    fig.reset_pose(pos=(W // 2, H // 2 - 40))

        # Schedule forces from JSON
        player.update(dt, fig.sensor_bodies())

        # Apply active forces (ONLY to sensor bodies)
        fm.update(dt)

        # Enforce constraints BEFORE stepping physics
        fig.enforce_limb_sides()
        fig.enforce_anti_flip()

        # Update yaw indicator from recently-applied sensor forces
        applied = fm.last_applied()
        fig.update_yaw_from_sensor_forces(applied)
        # Dampen yaw velocity a little each frame (stable spin)
        fig._yaw_vel *= pow(fig.cfg.yaw_damping, dt * 60)

        space.step(dt)

        # Render
        screen.fill((15, 18, 22))
        space.debug_draw(draw_options)

        # Overlay: draw torso "F/B" indicator centered on torso
        tx, ty = fig.torso.position
        label = fig.yaw_indicator()
        draw_text(screen, label, (int(tx) - 10, int(ty) - 18), size=28)

        # UI
        draw_text(screen, "SPACE: demo_spin   J: demo_jump   R: reset   ESC: quit", (18, 12), 20)
        draw_text(screen, f"Torso tilt: {math.degrees(fig.torso.angle): .1f}° (clamped ±{cfg.max_torso_tilt_deg}°)", (18, 40), 18)
        draw_text(screen, f"Yaw (simulated): {fig.yaw_deg: .1f}°   Facing: {label}", (18, 62), 18)
        draw_text(screen, "Hard rule: forces apply ONLY to sensors (no torso forces).", (18, 84), 18)

        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()
