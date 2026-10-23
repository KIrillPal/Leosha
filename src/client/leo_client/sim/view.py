"""Optional top-down window. It is not required to run the mock robot."""

from __future__ import annotations

import threading


def start_view(body) -> None:
    thread = threading.Thread(target=_loop, args=(body,), name="sim-view", daemon=True)
    thread.start()


def _loop(body) -> None:
    import pygame

    pygame.init()
    screen = pygame.display.set_mode((640, 480))
    pygame.display.set_caption("LEO2 mock")
    clock = pygame.time.Clock()
    scale = 70.0
    while not body.stop.is_set():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                body.stop.set()
        screen.fill((24, 24, 24))
        for segment in list(body.cfg["world"]["walls"]) + list(body.cfg["world"]["boxes"]):
            x1, y1, x2, y2 = segment
            pygame.draw.line(
                screen,
                (220, 220, 220),
                (int(x1 * scale), int(y1 * scale)),
                (int(x2 * scale), int(y2 * scale)),
                2,
            )
        x, y, yaw = body.pose()
        px, py = int(x * scale), int(y * scale)
        pygame.draw.circle(screen, (80, 180, 80), (px, py), 8)
        import math

        pygame.draw.line(screen, (80, 180, 80), (px, py), (int(px + math.cos(yaw) * 16), int(py + math.sin(yaw) * 16)), 2)
        pygame.display.flip()
        clock.tick(20)
    pygame.quit()
