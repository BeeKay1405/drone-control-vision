"""
drone_demo.py -- full simulator with keyboard control + dashboard.

Renders a 3D quadrotor that you can tilt/yaw with the keyboard. Each
60 Hz frame the controller produces motor commands which the physics
applies, closing the loop.

CONTROL BACKENDS:
  --software (default): use sage_math.control_tick() running in Python.
                        Useful for development and testing the simulator.
  --hardware           : use sage_hardware.control_tick() which calls the
                        PYNQ board over HTTP. Demonstrates FPGA control.

CONTROLS:
  I/K       pitch nose-down/up
  J/L       roll left/right
  A/D       yaw left/right
  R         reset to upright
  SPACE     toggle pilot input
  Q         quit

REQUIREMENTS:
    pip install pygame-ce numpy requests
"""

import argparse
import math
import sys
import time

import numpy as np
import pygame

from physics import Drone, quat_to_euler

# ---------------- webcam vision (added) ----------------
# Live camera feed is drawn INSIDE the pygame window (no second window, no
# event-loop clash with OpenCV). The frame grab happens every loop; pressing C
# fires a threaded inference to the SAME board/fabric via /infer_raw so the
# 60 Hz flight loop never blocks.
import os, struct, threading

try:
    import cv2
    _HAVE_CV2 = True
except Exception:
    _HAVE_CV2 = False
try:
    import requests as _rq
except Exception:
    _rq = None

_VIS = {
    "server": os.environ.get("DRONE_SERVER", "http://192.168.137.168:9100"),
    "digit": None, "ms": 0.0, "at": 0.0, "busy": False, "err": None,
    "cap": None, "last_frame": None, "cam_idx": 0,
}
_vlock = threading.Lock()


def vision_open_camera(idx=0):
    if not _HAVE_CV2:
        with _vlock: _VIS["err"] = "opencv-python not installed"
        return
    cap = cv2.VideoCapture(idx)
    if not cap.isOpened():
        with _vlock: _VIS["err"] = f"camera {idx} not found"
        return
    _VIS["cap"] = cap
    _VIS["cam_idx"] = idx


def vision_grab():
    """Grab one frame (called every main-loop iteration). Cheap; no inference."""
    cap = _VIS["cap"]
    if cap is None:
        return None
    ok, frame = cap.read()
    if not ok:
        return None
    _VIS["last_frame"] = frame
    return frame


def _vision_preprocess(frame_bgr):
    try:
        from preprocess import preprocess_frame
        # crop the centre square (matches the green box we draw) before preprocessing
        h, w = frame_bgr.shape[:2]
        s = min(h, w); y0, x0 = (h - s)//2, (w - s)//2
        crop = frame_bgr[y0:y0+s, x0:x0+s]
        q26, _ = preprocess_frame(crop, return_intermediate=True)
        return q26
    except Exception:
        g = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        h, w = g.shape; s = min(h, w); y0, x0 = (h-s)//2, (w-s)//2
        g = cv2.resize(g[y0:y0+s, x0:x0+s], (26, 26), interpolation=cv2.INTER_AREA)
        if g.mean() > 127: g = 255 - g
        q = np.round(g.astype(np.float32)/255.0*63.0 - 32.0)
        return np.clip(q, -32, 31).astype(np.int16)


def _vision_worker(frame_bgr):
    with _vlock:
        _VIS["busy"] = True; _VIS["err"] = None
    try:
        q26 = _vision_preprocess(frame_bgr)
        body = struct.pack("<676h", *q26.flatten().astype(np.int16).tolist())
        r = _rq.post(_VIS["server"] + "/infer_raw", data=body, timeout=5)
        r.raise_for_status()
        d = r.json()
        with _vlock:
            _VIS["digit"] = d["prediction"]; _VIS["ms"] = d.get("elapsed_ms", 0.0)
            _VIS["at"] = time.time()
    except Exception as e:
        with _vlock: _VIS["err"] = str(e)
    finally:
        with _vlock: _VIS["busy"] = False


def vision_classify_current():
    """Classify the most recent grabbed frame (non-blocking)."""
    if _rq is None:
        with _vlock: _VIS["err"] = "requests not installed"
        return
    frame = _VIS["last_frame"]
    if frame is None:
        with _vlock: _VIS["err"] = "no camera frame yet"
        return
    threading.Thread(target=_vision_worker, args=(frame.copy(),), daemon=True).start()


def draw_camera_panel(screen, font):
    """Render the live camera feed + green capture box INSIDE the pygame window,
    bottom-left. Also shows the last predicted digit."""
    frame = _VIS["last_frame"]
    PANEL_W, PANEL_H = 320, 240
    H = screen.get_size()[1]
    px, py = 12, H - PANEL_H - 12

    if frame is not None:
        # BGR(np) -> RGB -> pygame surface, scaled into the panel
        small = cv2.resize(frame, (PANEL_W, PANEL_H))
        rgb = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)
        surf = pygame.image.frombuffer(rgb.tobytes(), (PANEL_W, PANEL_H), "RGB")
        screen.blit(surf, (px, py))
        # green capture box = centre square of the panel
        s = min(PANEL_W, PANEL_H)
        bx = px + (PANEL_W - s)//2
        by = py + (PANEL_H - s)//2
        pygame.draw.rect(screen, (0, 230, 0), (bx, by, s, s), 2)
    else:
        pygame.draw.rect(screen, (30, 30, 40), (px, py, PANEL_W, PANEL_H))
        msg = "no camera (press V to open)"
        with _vlock: e = _VIS["err"]
        if e: msg = e[:34]
        screen.blit(font.render(msg, True, (220,160,160)), (px+10, py+PANEL_H//2))

    # status line under the panel
    with _vlock:
        d, ms, busy = _VIS["digit"], _VIS["ms"], _VIS["busy"]
    label = "classifying..." if busy else (
        f"digit: {d}   ({ms:.1f} ms on board)" if d is not None else "press C to classify")
    screen.blit(font.render(label, True, (0,255,255)), (px, py - 22))


# ---------------- 3D wireframe drone ----------------

ARM_LEN = 1.0
NOSE_LEN = 0.4
DRONE_POINTS = np.array([
    [ 0.0,  0.0,  0.0],
    [ ARM_LEN,  ARM_LEN, 0],
    [-ARM_LEN,  ARM_LEN, 0],
    [-ARM_LEN, -ARM_LEN, 0],
    [ ARM_LEN, -ARM_LEN, 0],
    [ NOSE_LEN, 0.0, 0.0],
])
ARMS = [(1, 3), (2, 4)]
MOTORS = [1, 2, 3, 4]
NOSE_LINE = (0, 5)


def quat_rotate_vec(q, v):
    w, x, y, z = q
    t = 2.0 * np.cross([x, y, z], v)
    return v + w * t + np.cross([x, y, z], t)


def project_3d_to_2d(p3d, screen_w, screen_h, scale=120, cam_dist=5.0):
    cam_y = -cam_dist
    cam_z = cam_dist * 0.5
    rx = p3d[0]
    ry = p3d[1] - cam_y
    rz = p3d[2] - cam_z
    pitch = math.atan2(cam_z, cam_dist)
    cp, sp = math.cos(-pitch), math.sin(-pitch)
    ry2 = ry * cp - rz * sp
    rz2 = ry * sp + rz * cp
    if ry2 < 0.1: ry2 = 0.1
    px = rx / ry2 * scale + screen_w / 2
    py = -rz2 / ry2 * scale + screen_h / 2
    return int(px), int(py)


# ---------------- dashboard rendering ----------------

def draw_dashboard(screen, font, drone, ctrl_result, motors, pilot_active,
                   q_int, omega_int, backend, latency_ms):
    W, H = screen.get_size()
    panel_x = W - 360
    panel_y = 20

    title = font.render(f"SAGE-16 demo [{backend}]", True, (240, 240, 240))
    screen.blit(title, (panel_x, panel_y));  panel_y += 32

    roll, pitch, yaw = (math.degrees(a) for a in quat_to_euler(drone.q))

    def line(label, val, color=(200, 220, 200)):
        nonlocal panel_y
        txt = font.render(f"{label}: {val}", True, color)
        screen.blit(txt, (panel_x, panel_y))
        panel_y += 22

    line("roll  (deg)",  f"{roll:+7.2f}", (200, 220, 255))
    line("pitch (deg)",  f"{pitch:+7.2f}", (255, 220, 200))
    line("yaw   (deg)",  f"{yaw:+7.2f}",   (200, 255, 220))
    panel_y += 8

    line("omega x (rad/s)", f"{drone.omega[0]:+6.3f}", (170, 200, 240))
    line("omega y (rad/s)", f"{drone.omega[1]:+6.3f}", (240, 200, 170))
    line("omega z (rad/s)", f"{drone.omega[2]:+6.3f}", (170, 240, 200))
    panel_y += 8

    sub = font.render("SAGE int16 inputs:", True, (180, 180, 200))
    screen.blit(sub, (panel_x, panel_y));  panel_y += 22
    line("  q  (w,x,y,z)", f"({q_int[0]:+d},{q_int[1]:+d},{q_int[2]:+d},{q_int[3]:+d})",
         (160, 180, 210))
    line("  omega (x,y,z)", f"({omega_int[0]:+d},{omega_int[1]:+d},{omega_int[2]:+d})",
         (160, 180, 210))
    panel_y += 8

    sub = font.render("SAGE int32 outputs:", True, (180, 180, 200))
    screen.blit(sub, (panel_x, panel_y));  panel_y += 22
    err = ctrl_result["error"]
    line("  err (w,x,y,z)", f"({err[0]:+d},{err[1]:+d},{err[2]:+d},{err[3]:+d})",
         (210, 180, 160))
    tau = ctrl_result["torque"]
    line("  torque (x,y,z)", f"({tau[0]:+d},{tau[1]:+d},{tau[2]:+d})",
         (210, 180, 160))
    panel_y += 8

    sub = font.render("Motor commands:", True, (180, 180, 200))
    screen.blit(sub, (panel_x, panel_y));  panel_y += 22
    for i, m in enumerate(motors):
        line(f"  motor {i}", f"{m:+8d}", (220, 220, 160))
    panel_y += 8

    bar_x = panel_x
    bar_y = panel_y
    bar_w = 320
    bar_h = 14
    max_m = max(1, max(abs(int(v)) for v in motors))
    for i, m in enumerate(motors):
        cx = bar_x + bar_w // 2
        cy = bar_y + i * (bar_h + 4)
        wpx = int((int(m) / max_m) * (bar_w / 2))
        color = (100, 200, 100) if int(m) >= 0 else (200, 100, 100)
        if wpx >= 0:
            pygame.draw.rect(screen, color, (cx, cy, wpx, bar_h))
        else:
            pygame.draw.rect(screen, color, (cx + wpx, cy, -wpx, bar_h))
        pygame.draw.rect(screen, (90, 90, 90), (bar_x, cy, bar_w, bar_h), 1)
    panel_y = bar_y + 4 * (bar_h + 4) + 8

    # Latency / loop status
    lat_col = (160, 240, 160) if latency_ms < 20 else (240, 240, 160) if latency_ms < 60 else (240, 160, 160)
    line(f"loop latency", f"{latency_ms:6.1f} ms", lat_col)
    panel_y += 4

    if pilot_active:
        msg = "PILOT INPUT ACTIVE (SPACE to release)"
        col = (255, 220, 100)
    else:
        msg = "controller alone (hold I/K/J/L/A/D to fly)"
        col = (160, 160, 180)
    pygame.draw.rect(screen, (40, 40, 50), (panel_x, panel_y, 340, 24))
    txt = font.render(msg, True, col)
    screen.blit(txt, (panel_x + 5, panel_y + 4))


def draw_help(screen, font):
    H = screen.get_size()[1]
    hints = [
        "I/K  pitch  nose-down/up",
        "J/L  roll   left/right",
        "A/D  yaw    left/right",
        "R    reset drone",
        "SPACE  toggle pilot",
        "C    classify digit   V  open camera",
        "Q    quit",
    ]
    y = H - len(hints) * 18 - 10
    for h in hints:
        t = font.render(h, True, (140, 140, 160))
        screen.blit(t, (12, y));  y += 18


# ---------------- main loop ----------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hardware", action="store_true",
                    help="use the PYNQ FPGA over HTTP instead of local Python")
    ap.add_argument("--server", default=None,
                    help="override the server URL (default reads DRONE_SERVER env, "
                         "else http://192.168.137.168:9100)")
    args = ap.parse_args()

    if args.hardware:
        if args.server:
            import os
            os.environ["DRONE_SERVER"] = args.server
        from sage_hardware import control_tick
        backend = "hardware"
    if args.server:
        _VIS["server"] = args.server
    else:
        from sage_math import control_tick
        backend = "software"

    pygame.init()
    pygame.display.set_caption(f"SAGE-16 attitude-hold demo [{backend}]")
    screen = pygame.display.set_mode((1280, 720))
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Consolas", 16)

    drone = Drone()
    pilot_active = True
    thrust_cmd = 1000
    PILOT_TORQUE = 0.6
    DT = 1.0 / 60.0

    # rolling latency average
    lat_ema = 0.0

    if _HAVE_CV2:
        vision_open_camera(0)   # try to open webcam at startup

    running = True
    while running:
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False
            elif e.type == pygame.KEYDOWN:
                if e.key == pygame.K_q:
                    running = False
                elif e.key == pygame.K_r:
                    drone.reset()
                elif e.key == pygame.K_SPACE:
                    pilot_active = not pilot_active
                elif e.key == pygame.K_c:
                    vision_classify_current()
                elif e.key == pygame.K_v:
                    vision_open_camera(_VIS["cam_idx"])

        vision_grab()
        keys = pygame.key.get_pressed()
        pt = np.zeros(3)
        if pilot_active:
            if keys[pygame.K_i]: pt[1] -= PILOT_TORQUE
            if keys[pygame.K_k]: pt[1] += PILOT_TORQUE
            if keys[pygame.K_j]: pt[0] -= PILOT_TORQUE
            if keys[pygame.K_l]: pt[0] += PILOT_TORQUE
            if keys[pygame.K_a]: pt[2] += PILOT_TORQUE
            if keys[pygame.K_d]: pt[2] -= PILOT_TORQUE
        drone.pilot_torque = pt

        q_int, omega_int = drone.get_sage_inputs()
        t0 = time.time()
        result = control_tick(q_int, omega_int, thrust_cmd)
        elapsed_ms = (time.time() - t0) * 1000
        lat_ema = 0.9 * lat_ema + 0.1 * elapsed_ms
        motors = result["motors"]
        drone.step(motors, dt=DT)

        screen.fill((18, 18, 24))

        for gx in range(-4, 5):
            p1 = project_3d_to_2d(np.array([gx, -4, 0.0]), 920, 720)
            p2 = project_3d_to_2d(np.array([gx,  4, 0.0]), 920, 720)
            pygame.draw.line(screen, (40, 40, 55), p1, p2, 1)
        for gy in range(-4, 5):
            p1 = project_3d_to_2d(np.array([-4, gy, 0.0]), 920, 720)
            p2 = project_3d_to_2d(np.array([ 4, gy, 0.0]), 920, 720)
            pygame.draw.line(screen, (40, 40, 55), p1, p2, 1)

        rotated = np.array([quat_rotate_vec(drone.q, p) for p in DRONE_POINTS])
        screen_pts = [project_3d_to_2d(p, 920, 720) for p in rotated]

        for (a, b) in ARMS:
            pygame.draw.line(screen, (180, 180, 200),
                             screen_pts[a], screen_pts[b], 4)
        motor_colors = [(220, 100, 100), (100, 220, 100),
                        (100, 100, 220), (220, 220, 100)]
        for i, mi in enumerate(MOTORS):
            mag = max(0, min(1, (int(motors[i]) + 1000) / 4000.0))
            base = motor_colors[i]
            color = tuple(int(c * (0.4 + 0.6 * mag)) for c in base)
            radius = 8 + int(mag * 8)
            pygame.draw.circle(screen, color, screen_pts[mi], radius)
        pygame.draw.line(screen, (255, 240, 120),
                         screen_pts[NOSE_LINE[0]], screen_pts[NOSE_LINE[1]], 3)
        pygame.draw.circle(screen, (240, 240, 240), screen_pts[0], 4)

        draw_dashboard(screen, font, drone, result, motors,
                       pilot_active, q_int, omega_int, backend, lat_ema)
        draw_help(screen, font)
        draw_camera_panel(screen, font)

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()


if __name__ == "__main__":
    main()
