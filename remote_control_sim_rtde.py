import socket
import rtde.rtde as rtde
from math import cos, sin, pi, sqrt
from pynput.keyboard import Listener, Key
from queue import Queue
from threading import Thread, Lock
from time import sleep

ROBOT_IP = "172.17.0.2"
RTDE_PORT = 30004
SCRIPT_PORT = 30002
STEP = 0.05
FREE_STEP = 0.10
SPEED_TOL = 0.001
RADIUS = 0.1

# ---------------------------------------------------------------------------
# RTDE setup — reads TCP pose + joint speeds; writes to input registers
# ---------------------------------------------------------------------------
con = rtde.RTDE(ROBOT_IP, RTDE_PORT)
con.connect()
con.get_controller_version()
con.send_output_setup(("actual_TCP_pose", "actual_qd"), ("VECTOR6D", "VECTOR6D"))
reg_names = tuple(f"input_double_register_{i}" for i in range(6))
reg = con.send_input_setup(reg_names, ("DOUBLE",) * 6)
con.send_start()


def send(cmd):
    s = socket.create_connection((ROBOT_IP, SCRIPT_PORT))
    s.sendall(cmd.encode() + b"\n")
    s.close()


def stop_robot():
    send("stopj(3.0)")


def move_to(x, y, z, rx, ry, rz):
    send(f"movej(p[{x},{y},{z},{rx},{ry},{rz}], a=0.5, v=0.3)")


def set_reg(*vals):
    for i, v in enumerate(vals):
        setattr(reg, f"input_double_register_{i}", v)
    con.send(reg)


def stopped(qd):
    return all(abs(v) < SPEED_TOL for v in qd)


def shapes(c, n):
    cx, cy, cz, rx, ry, rz = c
    if n == 1:  # circle
        return [(cx, cy + RADIUS * sin(2 * pi * i / 36), cz + RADIUS * cos(2 * pi * i / 36), rx, ry, rz) for i in range(36)]
    if n == 2:  # ping-pong
        return [(cx, cy + 0.1, cz, rx, ry, rz), (cx, cy - 0.1, cz, rx, ry, rz)]
    if n == 3:  # triangle
        return [(cx, cy + RADIUS * sin(2 * pi * i / 3), cz + RADIUS * cos(2 * pi * i / 3), rx, ry, rz) for i in range(3)]
    sq = []  # square
    for dy, dz in [(0.1, 0.1), (0.1, -0.1), (-0.1, -0.1), (-0.1, 0.1)]:
        sq.append((cx, cy + dy, cz + dz, rx, ry, rz))
    return sq


# ---------------------------------------------------------------------------
# Shared state  (protected by lock)
# ---------------------------------------------------------------------------
lock = Lock()
current_tcp = [0.0] * 6       # latest TCP pose (main thread writes, others read)
current_qd = [0.0] * 6        # latest joint speeds (main thread writes, others read)
mode = "free"                 # "free" | "shape" | "pause"
wps = []                      # list of waypoints for the current shape
widx = 0                      # index of the next waypoint to send
offset = [0.0, 0.0, 0.0]      # user adjustment offset (via arrow keys)
running = True                # quit flag


# ---------------------------------------------------------------------------
# Movement thread — runs shape routines in a continuous loop
# ---------------------------------------------------------------------------
def movement_loop():
    global widx
    while running:
        # snapshot shared state under lock
        with lock:
            m = mode
            has_wps = bool(wps)
            total = len(wps)
            wi = widx
            off = list(offset)

        if m != "shape" or not has_wps:
            sleep(0.1)
            continue

        # wrap around for continuous looping
        if wi >= total:
            with lock:
                if wps and widx >= len(wps):
                    widx = 0
                    wi = 0
                else:
                    continue

        # read the next waypoint + offset atomically
        with lock:
            wi = widx
            off = list(offset)
            if not wps or wi >= len(wps):
                continue
            w = wps[wi]               # <--- read from live wps, not a stale copy

        tx, ty, tz = w[0] + off[0], w[1] + off[1], w[2] + off[2]

        move_to(tx, ty, tz, w[3], w[4], w[5])
        set_reg(tx, ty, tz, w[3], w[4], w[5])

        with lock:
            widx += 1

        # wait until the robot reaches this target (or is paused/cancelled)
        while running:
            with lock:
                m = mode
                if m in ("shape", "pause") and wps:
                    wi = widx
                    off = list(offset)
                else:
                    wi = -1  # signal "no valid state"

            if wi < 0:
                break

            if m == "pause":
                # inner sleep loop — blocks here until resume
                while running:
                    with lock:
                        m = mode
                    if m != "pause":
                        break
                    sleep(0.05)
                if m == "shape":
                    # re-read waypoint + offset for the interrupted move
                    with lock:
                        i = (widx - 1) if widx > 0 else 0
                        if not wps or i >= len(wps):
                            continue
                        w = wps[i]
                        off = list(offset)
                    tx, ty, tz = w[0] + off[0], w[1] + off[1], w[2] + off[2]
                    move_to(tx, ty, tz, w[3], w[4], w[5])
                    set_reg(tx, ty, tz, w[3], w[4], w[5])
                continue

            if m != "shape":
                break

            # detect offset changes mid-flight and override the current move
            with lock:
                i = (widx - 1) if widx > 0 else 0
                if not wps or i >= len(wps):
                    break
                w = wps[i]
                off = list(offset)
            new_tx, new_ty, new_tz = w[0] + off[0], w[1] + off[1], w[2] + off[2]
            if abs(new_tx - tx) > 0.0001 or abs(new_ty - ty) > 0.0001 or abs(new_tz - tz) > 0.0001:
                tx, ty, tz = new_tx, new_ty, new_tz
                move_to(tx, ty, tz, w[3], w[4], w[5])
                set_reg(tx, ty, tz, w[3], w[4], w[5])

            cx, cy, cz = current_tcp[:3]
            if sqrt((cx - tx)**2 + (cy - ty)**2 + (cz - tz)**2) < 0.005:
                break
            sleep(0.05)


# ---------------------------------------------------------------------------
# Free-move thread — arrow-key movement when no routine is active
# ---------------------------------------------------------------------------
free_q = Queue()


def free_move_loop():
    while running:
        try:
            dx, dy, dz = free_q.get(timeout=0.2)
        except:
            continue
        with lock:
            m = mode
            qd = list(current_qd)
            tcp = list(current_tcp)
        if m != "free" or not stopped(qd):
            continue
        cx, cy, cz, rx, ry, rz = tcp
        tx, ty, tz = cx + dx, cy + dy, cz + dz
        print(f"\nFree move to: x={tx:.3f} y={ty:.3f} z={tz:.3f}")
        set_reg(tx, ty, tz, rx, ry, rz)
        move_to(tx, ty, tz, rx, ry, rz)


Thread(target=movement_loop, daemon=True).start()
Thread(target=free_move_loop, daemon=True).start()


# ---------------------------------------------------------------------------
# Keyboard listener — dispatches commands to the main queue
# ---------------------------------------------------------------------------
q = Queue()


def on_press(key):
    global running
    if key == Key.up:
        q.put(("arrow", 0, 1, 0))
    elif key == Key.down:
        q.put(("arrow", 0, -1, 0))
    elif key == Key.left:
        q.put(("arrow", -1, 0, 0))
    elif key == Key.right:
        q.put(("arrow", 1, 0, 0))
    elif key == Key.space:
        q.put(("space",))
    elif key == Key.esc:
        q.put(("quit",))
    else:
        try:
            c = key.char
            if c == '+':
                q.put(("arrow", 0, 0, 1))
            elif c == '-':
                q.put(("arrow", 0, 0, -1))
            elif c == 'q':
                q.put(("quit",))
            elif c == 'c':
                q.put(("cancel",))
            elif c in '1234':
                q.put(("shape", int(c)))
        except AttributeError:
            pass


# ---------------------------------------------------------------------------
# Main loop — RTDE receive + command processing
# ---------------------------------------------------------------------------
with Listener(on_press=on_press) as listener:
    while running and listener.is_alive():
        state = con.receive()
        if state is None:
            break
        tcp, qd = state.actual_TCP_pose, state.actual_qd
        current_tcp[:] = tcp
        current_qd[:] = qd
        x, y, z, rx, ry, rz = tcp

        with lock:
            m = mode
            wi = widx
            wp_len = len(wps)

        print(f"\rx={x:.3f} y={y:.3f} z={z:.3f}  mode={m}  wp={wi}/{wp_len}", end="")

        while not q.empty():
            cmd = q.get_nowait()

            if cmd[0] == "quit":
                running = False

            elif cmd[0] == "cancel":
                should_stop = False
                with lock:
                    if mode in ("shape", "pause"):
                        mode = "free"
                        should_stop = True
                if should_stop:
                    stop_robot()

            elif cmd[0] == "shape":
                with lock:
                    if mode != "free":
                        continue
                if not stopped(qd):
                    continue
                with lock:
                    wps = shapes((x, y, z, rx, ry, rz), cmd[1])
                    widx = 0
                    offset = [0.0, 0.0, 0.0]
                    mode = "shape"

            elif cmd[0] == "space":
                should_stop = False
                with lock:
                    if mode == "shape":
                        mode = "pause"
                        should_stop = True
                    elif mode == "pause":
                        mode = "shape"
                if should_stop:
                    stop_robot()

            elif cmd[0] == "arrow":
                with lock:
                    m = mode
                    if m in ("shape", "pause"):
                        offset[0] += cmd[1] * STEP
                        offset[1] += cmd[2] * STEP
                        offset[2] += cmd[3] * STEP
                        i = (widx - 1) if widx > 0 else 0
                        if wps:
                            w = wps[i]
                            set_reg(
                                w[0] + offset[0],
                                w[1] + offset[1],
                                w[2] + offset[2],
                                w[3], w[4], w[5]
                            )
                    free = (m == "free")

                if m in ("shape", "pause"):
                    if m == "shape" and wps:
                        with lock:
                            i = (widx - 1) if widx > 0 else 0
                            w = wps[i]
                            tx, ty, tz = w[0] + offset[0], w[1] + offset[1], w[2] + offset[2]
                        move_to(tx, ty, tz, w[3], w[4], w[5])
                    with lock:
                        print(f"\nOffset: ({offset[0]:.3f}, {offset[1]:.3f}, {offset[2]:.3f})")
                elif free:
                    free_q.put((cmd[1] * FREE_STEP, cmd[2] * FREE_STEP, cmd[3] * FREE_STEP))

con.send_pause()
con.disconnect()
