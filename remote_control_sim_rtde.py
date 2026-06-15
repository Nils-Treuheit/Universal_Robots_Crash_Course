import socket
import rtde.rtde as rtde
from math import cos, sin, pi, sqrt

ROBOT_IP = "172.17.0.2"
RTDE_PORT = 30004
SCRIPT_PORT = 30002
STEP = 0.05
FREE_STEP = 0.10
SPEED_TOL = 0.001
RADIUS = 0.1

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
    if n == 1:
        return [(cx + RADIUS * cos(2 * pi * i / 36), cy + RADIUS * sin(2 * pi * i / 36), cz, rx, ry, rz) for i in range(36)]
    if n == 2:
        return [(cx + 0.1, cy, cz, rx, ry, rz), (cx - 0.1, cy, cz, rx, ry, rz)]
    if n == 3:
        return [(cx + RADIUS * cos(2 * pi * i / 3), cy + RADIUS * sin(2 * pi * i / 3), cz, rx, ry, rz) for i in range(3)]
    sq = []
    for dx, dy in [(1, 1), (1, -1), (-1, -1), (-1, 1)]:
        sq.append((cx + dx * RADIUS, cy + dy * RADIUS, cz, rx, ry, rz))
    return sq


from pynput.keyboard import Listener, Key
from queue import Queue

q = Queue()
mode = "free"
wps = []
widx = 0
offset = [0, 0, 0]
running = True
target = None


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
            elif c in '1234':
                q.put(("shape", int(c)))
        except AttributeError:
            pass


with Listener(on_press=on_press) as listener:
    while running and listener.is_alive():
        state = con.receive()
        if state is None:
            break
        tcp, qd = state.actual_TCP_pose, state.actual_qd
        x, y, z, rx, ry, rz = tcp
        print(f"\rx={x:.3f} y={y:.3f} z={z:.3f}  mode={mode}  wp={widx}/{len(wps)}", end="")

        while not q.empty():
            cmd = q.get_nowait()

            if cmd[0] == "quit":
                running = False

            elif cmd[0] == "shape":
                if not stopped(qd):
                    continue
                wps = shapes((x, y, z, rx, ry, rz), cmd[1])
                widx = 0
                offset = [0, 0, 0]
                mode = "shape"
                if wps:
                    set_reg(*wps[0])
                    move_to(*wps[0])
                    target = wps[0][:3]
                    widx = 1

            elif cmd[0] == "space":
                if mode == "shape":
                    stop_robot()
                    mode = "pause"
                elif mode == "pause":
                    if widx < len(wps):
                        for i in range(widx, len(wps)):
                            w = wps[i]
                            wps[i] = (w[0] + offset[0], w[1] + offset[1], w[2] + offset[2], w[3], w[4], w[5])
                        offset = [0, 0, 0]
                        set_reg(*wps[widx])
                        move_to(*wps[widx])
                        target = wps[widx][:3]
                        widx += 1
                        mode = "shape"

            elif cmd[0] == "arrow":
                if mode == "pause":
                    offset[0] += cmd[1] * STEP
                    offset[1] += cmd[2] * STEP
                    offset[2] += cmd[3] * STEP
                    if widx < len(wps):
                        tx = wps[widx][0] + offset[0]
                        ty = wps[widx][1] + offset[1]
                        tz = wps[widx][2] + offset[2]
                        set_reg(tx, ty, tz, wps[widx][3], wps[widx][4], wps[widx][5])
                    print(f"\nAdjusted target by ({offset[0]:.3f}, {offset[1]:.3f}, {offset[2]:.3f})")
                elif mode == "free" and stopped(qd):
                    tx, ty, tz = x + cmd[1] * FREE_STEP, y + cmd[2] * FREE_STEP, z + cmd[3] * FREE_STEP
                    print(f"\nFree move to: x={tx:.3f} y={ty:.3f} z={tz:.3f}")
                    set_reg(tx, ty, tz, rx, ry, rz)
                    move_to(tx, ty, tz, rx, ry, rz)
                    target = (tx, ty, tz)

        if mode == "shape" and widx < len(wps):
            if target is None or sqrt((x - target[0])**2 + (y - target[1])**2 + (z - target[2])**2) < 0.005:
                set_reg(*wps[widx])
                move_to(*wps[widx])
                target = wps[widx][:3]
                widx += 1
        elif mode == "shape" and widx >= len(wps):
            if target is None or sqrt((x - target[0])**2 + (y - target[1])**2 + (z - target[2])**2) < 0.005:
                mode = "free"
                target = None
                print("\nRoutine complete")

con.send_pause()
con.disconnect()
