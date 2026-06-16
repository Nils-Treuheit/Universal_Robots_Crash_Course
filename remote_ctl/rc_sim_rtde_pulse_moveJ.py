import socket
import rtde.rtde as rtde
from math import cos, sin, pi, sqrt
from pynput.keyboard import Listener, Key
from queue import Queue
from threading import Thread, Lock
from time import sleep, time

ROBOT_IP = "172.17.0.2"
SCRIPT_SOCKET = None
RTDE_PORT = 30004
STEP = 0.05
FREE_STEP = 0.10
SPEED_TOL = 0.001
DIST_TOL = 0.01
RADIUS = 0.1

# ---------------------------------------------------------------------------
# RTDE setup
# Reads TCP pose + joint velocities + output_int_register_0 (robot ack).
# Writes to 6 input double registers (waypoint) + input_int_register_0 (flag).
# ---------------------------------------------------------------------------
con = rtde.RTDE(ROBOT_IP, RTDE_PORT)
con.connect()
con.get_controller_version()
con.send_output_setup(
    ("actual_TCP_pose", "actual_qd", "output_int_register_0"),
    ("VECTOR6D", "VECTOR6D", "INT32"),
)
reg_names = tuple(f"input_double_register_{i}" for i in range(6))
reg = con.send_input_setup(reg_names, ("DOUBLE",) * 6)
int_reg = con.send_input_setup(("input_int_register_0",), ("INT32",))
con.send_start()

def send_move_to_RTDE(s:socket):
    """
    Send a movej command to the robot controller via port 30002 that reads
    the target pose from the input registers (set by RTDE).  The ``r=0.02``
    blend radius prevents the robot from coming to a full stop at each
    waypoint, producing smooth continuous motion between consecutive moves.
    """
    cmd = (
        "movej(p[read_input_float_register(0), read_input_float_register(1),"
        "read_input_float_register(2), read_input_float_register(3),"
        "read_input_float_register(4), read_input_float_register(5)],"
        "a=1.0, v=0.5, r=0.02)"
    )
    s.sendall(cmd.encode() + b"\n")


def stop_robot(s:socket):
    """
    Send a URScript snippet that stops the robot with a harsh deceleration
    and sets the ``running`` flag to False so the background loop
    terminates.
    """
    cmd = "stopj(3.0)"
    s.sendall(cmd.encode() + b"\n")

# ---------------------------------------------------------------------------
# Thread-safe shared state
# ---------------------------------------------------------------------------
waypoint_queue = Queue()
free_move_queue = Queue()
rtde_lock = Lock()
offset_lock = Lock()
state_lock = Lock()
current_tcp = [0.0] * 6
offset = [0.0] * 6
halted = False
paused = False
robot_moving = False
running = True
movement_routine = 0
waypoint_total = 0
waypoint_index = 0

# ---------------------------------------------------------------------------
# RTDE write helpers
# ---------------------------------------------------------------------------
def _set_flag():
    """Set ``input_int_register_0 = 1`` to signal a new waypoint."""
    int_reg.input_int_register_0 = 1
    con.send(int_reg)

def _clear_flag():
    """Clear ``input_int_register_0 = 0`` after the robot has ack'd."""
    int_reg.input_int_register_0 = 0
    con.send(int_reg)

def write_target(pose):
    """
    Write a 6-DOF pose to the input double registers and raise the
    ``input_int_register_0`` flag so the background script picks it up.
    """
    for i in range(6):
        setattr(reg, f"input_double_register_{i}", pose[i])
    con.send(reg)
    _set_flag()

# ---------------------------------------------------------------------------
# Shape generators (y-z plane, centred at *center*)
# ---------------------------------------------------------------------------
def generate_circle(center):
    """Return 36 waypoints forming a circle (radius ``RADIUS``) in the
    y-z plane around *center*."""
    pts = []
    for i in range(36):
        theta = 2 * pi * i / 36
        pt = list(center)
        pt[1] = center[1] + RADIUS * cos(theta)
        pt[2] = center[2] + RADIUS * sin(theta)
        pts.append(pt)
    return pts

def generate_line(center):
    """Return two waypoints — a vertical line of length ``2*RADIUS``
    through *center* along z."""
    pt1 = list(center)
    pt1[2] = center[2] + RADIUS
    pt2 = list(center)
    pt2[2] = center[2] - RADIUS
    return [pt1, pt2]

def generate_triangle(center):
    """Return three waypoints — an equilateral triangle (circumradius
    ``RADIUS``) around *center* in the y-z plane."""
    pts = []
    for i in range(3):
        theta = 2 * pi * i / 3
        pt = list(center)
        pt[1] = center[1] + RADIUS * cos(theta)
        pt[2] = center[2] + RADIUS * sin(theta)
        pts.append(pt)
    return pts

def generate_square(center):
    """Return four waypoints — a square (circumradius ``RADIUS``,
    rotated 45°) around *center* in the y-z plane."""
    pts = []
    for i in range(4):
        theta = 2 * pi * i / 4 + pi / 4
        pt = list(center)
        pt[1] = center[1] + RADIUS * cos(theta)
        pt[2] = center[2] + RADIUS * sin(theta)
        pts.append(pt)
    return pts

# ---------------------------------------------------------------------------
# Mother thread — movement planner with integrated real-time feedback
# ---------------------------------------------------------------------------
def movement_planner():
    """
    Main control loop running at ≈100 Hz.

    Each iteration:
      1. Receives the latest RTDE state (TCP pose, joint speeds, ack).
      2. Prints a continuously-overwritten status line (``\\r``) showing
         current TCP pose, target pose, movement speed, and waypoint
         progress (e.g. ``wp: 3/36``).
      3. Handles halt requests by draining the waypoint queue.
      4. If the robot is stationary and waypoints remain, sends the next
         waypoint via ``write_target()`` and waits for the background
         script to acknowledge via ``output_int_register_0`` (0→1→0
         transition).  After ``movej`` completes, the input flag is
         cleared so the same waypoint is not replayed.
    """
    global robot_moving, offset, halted, waypoint_index, waypoint_total, current_tcp, paused
    tick = 0
    while running:
        # --- read RTDE state ------------------------------------------------
        with rtde_lock:
            state = con.receive()
        if state is None:
            sleep(0.01)
            continue
        # > got RTDE state
        current_tcp = list(state.actual_TCP_pose)
        qd = state.actual_qd
        if current_tcp is None or qd is None:
            sleep(0.01)
            continue
        speed = sqrt(sum(s * s for s in qd))
        moving = speed > SPEED_TOL
        robot_ack = state.output_int_register_0
        # > got TCP and speed from RTDE
        with state_lock:
            robot_moving = moving
            wi = waypoint_index
            wt = waypoint_total
            mr = movement_routine
        # > set global thread safe state vars

        # --- calculate target dist (if prev target) --------------------------
        target = [getattr(reg, f"input_double_register_{i}") for i in range(6)]
        if any(v is None for v in target): dist2tgt = None
        else: dist2tgt = sqrt(sum([(c-t)**2 for c,t in zip(current_tcp,target)]))
        
        # --- halt handling --------------------------------------------------
        with state_lock:
            if halted:
                while not waypoint_queue.empty():
                    try:
                        waypoint_queue.get_nowait()
                    except Exception:
                        pass
                halted = False
                waypoint_index = 0
                waypoint_total = 0
        # > reported current state on cli

        # --- send next waypoint if idle & pending ---------------------------
        if not(moving) and not(waypoint_queue.empty()) and \
            ((dist2tgt is None and (waypoint_index==0)) or (dist2tgt<DIST_TOL)):
            wp = list(waypoint_queue.get_nowait())
            waypoint_queue.put_nowait(wp)
            with state_lock:
                waypoint_index += 1
                waypoint_index = waypoint_index % waypoint_total

            with offset_lock:
                if any(abs(v) > 1e-6 for v in offset):
                    for i in range(6):
                        wp[i] += offset[i]
                    offset[:] = [0.0] * 6

            with rtde_lock:
                write_target(wp)

            print(
                f"moving to [{wp[0]:.4f}, {wp[1]:.4f}, {wp[2]:.4f}, "
                f"{wp[3]:.4f}, {wp[4]:.4f}, {wp[5]:.4f}]      "
            )

            # Wait for robot to ack (output goes 0→1→0).  The 1→0
            # transition signals that ``movej`` has started.
            deadline = time() + 1.0
            ack_started = False
            while running and time() < deadline:
                with rtde_lock: state = con.receive()
                if state is None:
                    sleep(0.001)
                    continue

                robot_ack = state.output_int_register_0
                if robot_ack == 1: ack_started = True
                if ack_started and robot_ack == 0:
                    with rtde_lock: _clear_flag()
                    break
                sleep(0.001)
            else:
                # Timed out – clear flag so robot doesn't retry.
                with rtde_lock: _clear_flag()

            sleep(0.001)

        # --- send movej if idle ----------------------------------------------
        if not moving and not paused and not halted:
            send_move_to_RTDE(SCRIPT_SOCKET)

        # --- real-time status line ------------------------------------------
        with state_lock:
            robot_moving = moving
            wi = waypoint_index
            wt = waypoint_total
            mr = movement_routine
        # > update global thread safe state vars
        target = [getattr(reg, f"input_double_register_{i}") for i in range(6)]
        if any(v is None for v in target):
            tgt_str = "  [unset]"
            dist2tgt_str = "[no_tgt]"
            dist2tgt = None
        else:
            dist2tgt = sqrt(sum([(c-t)**2 for c,t in zip(current_tcp,target)]))
            dist2tgt_str = f"{dist2tgt:8.4f}"
            tgt_str = (f"[{target[0]:8.4f} {target[1]:8.4f} {target[2]:8.4f} "
                       f"{target[3]:6.2f} {target[4]:6.2f} {target[5]:6.2f}]")
        routine_str = ""
        if mr != 0: routine_str = f"| routine {mr} pending"
        elif wt > 0: routine_str = f"| wp: {wi}/{wt}"
        print(
            "\r",
            f"tcp=[{current_tcp[0]:8.4f} {current_tcp[1]:8.4f} {current_tcp[2]:8.4f}",
            f"{current_tcp[3]:6.2f} {current_tcp[4]:6.2f} {current_tcp[5]:6.2f}]",
            f"tgt={tgt_str} spd={speed:8.4f} dist={dist2tgt_str} ack={robot_ack}",
            f"{routine_str}  prog_tick={tick:8d}",
            end="\r",
            flush=True,
        )
        tick += 1
        tick = tick % 10000
        # > reported current state on cli
        sleep(0.01)

# ---------------------------------------------------------------------------
# Free-movement thread
# ---------------------------------------------------------------------------
def free_movement():
    """
    Consumes incremental-motion deltas from ``free_move_queue`` (produced
    by keyboard arrows / +/-).  Behaviour depends on robot state:

    * **Robot moving** — applies the delta directly to the current target
      pose (live steering).
    * **Robot idle** — creates a new target waypoint at the offset
      TCP position it via RTDE.
    * **Robot idle, active routine** — accumulates the delta into the
      global *offset* so the next shape waypoint is shifted.
    """
    global offset, robot_moving, current_tcp
    while running:
        moving = False
        try: delta = free_move_queue.get(timeout=0.05)
        except Exception: continue
        with state_lock: moving = robot_moving
        if moving:
            print("[FreeMoveThread] Manipulate Movement!")
            cur_tgt = [getattr(reg, f"input_double_register_{i}") for i in range(6)]
            for i in range(3): cur_tgt[i] += delta[i]
            with rtde_lock: write_target(cur_tgt)
        elif waypoint_queue.empty():
            print("[FreeMoveThread] Manipulate Target WP!")
            with rtde_lock:
                for i in range(3): current_tcp[i] += delta[i]
                write_target(current_tcp)
        elif not halted:
            print("[FreeMoveThread] Record Offset!")
            with offset_lock:
                for i in range(3): offset[i] += delta[i]

# ---------------------------------------------------------------------------
# Movement scheduler thread
# ---------------------------------------------------------------------------
def movement_scheduler():
    """
    Monitors ``movement_routine``.  When non-zero and the robot is idle
    with an empty waypoint queue, generates the corresponding shape's
    waypoints and enqueues them.  Resets ``movement_routine`` to 0
    immediately after.
    """
    global movement_routine, waypoint_total, waypoint_index, current_tcp
    generators = {1: generate_circle, 2: generate_line,
                  3: generate_triangle, 4: generate_square}
    while running:
        moving = True
        mr = 0
        with state_lock: 
            mr = movement_routine
            moving = robot_moving
        if mr != 0 and not(moving) and waypoint_queue.empty():
            center = current_tcp
            pts = generators[mr](center)
            for pt in pts: waypoint_queue.put(pt)
            waypoint_total = len(pts)
            waypoint_index = 0
            movement_routine = 0
            print(f"\nmovement routine {mr} triggered ({waypoint_total} waypoints)")
        sleep(0.05)

# ---------------------------------------------------------------------------
# Keyboard listener
# ---------------------------------------------------------------------------
def on_press(key):
    """
    pynput callback mapping keys to actions:

    ========= ====================================
    Key       Action
    ========= ====================================
    Arrows    Jog TCP in ±X / ±Y (``FREE_STEP``)
    + / -     Jog TCP in ±Z
    Space     Pause / resume robot
    q         Quit (stop robot, exit)
    h         Halt (drain queue, stop robot)
    1–4       Trigger shape routine (circle / line / triangle / square)
    ========= ====================================
    """
    global running, paused, halted, movement_routine
    try:
        if key == Key.up:
            print(f"\nkey pressed: up")
            free_move_queue.put([-FREE_STEP, 0.0, 0.0, 0.0, 0.0, 0.0])
        elif key == Key.down:
            print(f"\nkey pressed: down")
            free_move_queue.put([FREE_STEP, 0.0, 0.0, 0.0, 0.0, 0.0])
        elif key == Key.left:
            print(f"\nkey pressed: left")
            free_move_queue.put([0.0, -FREE_STEP, 0.0, 0.0, 0.0, 0.0])
        elif key == Key.right:
            print(f"\nkey pressed: right")
            free_move_queue.put([0.0, FREE_STEP, 0.0, 0.0, 0.0, 0.0])
        elif key == Key.space:
            with state_lock:
                if paused:
                    print(f"\nstart event: resume (start move to RTDE pos)")
                    paused = False
                else:
                    print(f"\nstop event: pause (stop any current movement)")
                    stop_robot(SCRIPT_SOCKET)
                    paused = True
        elif hasattr(key, 'char') and key.char == 'q':
            print(f"\nkey pressed: q — terminating (end control program)")
            running = False
            stop_robot(SCRIPT_SOCKET)
            return False
        elif hasattr(key, 'char') and key.char == 'h':
            print(f"\nkey pressed: h — stop event (halt movement routine)")
            with state_lock:
                halted = True
            stop_robot(SCRIPT_SOCKET)
        elif hasattr(key, 'char') and key.char == '+':
            print(f"\nkey pressed: +")
            free_move_queue.put([0.0, 0.0, FREE_STEP, 0.0, 0.0, 0.0])
        elif hasattr(key, 'char') and key.char == '-':
            print(f"\nkey pressed: -")
            free_move_queue.put([0.0, 0.0, -FREE_STEP, 0.0, 0.0, 0.0])
        elif hasattr(key, 'char') and key.char in '1234':
            print(f"\nkey pressed: {key.char} (start movement routine)")
            with state_lock:
                if not robot_moving:
                    movement_routine = int(key.char)
    except AttributeError:
        pass

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
SCRIPT_SOCKET = socket.create_connection((ROBOT_IP, 30002))
init = False
while not(init==True):
    with rtde_lock:
        state = con.receive()
    if state is None:
        sleep(0.01)
        continue
    # > got RTDE state
    current_tcp = list(state.actual_TCP_pose)
    if current_tcp is None:
        sleep(0.01)
        continue
    current_tcp = list(state.actual_TCP_pose)
    write_target(current_tcp)
    init = True

threads = [
    Thread(target=movement_planner, daemon=True),
    Thread(target=free_movement, daemon=True),
    Thread(target=movement_scheduler, daemon=True),
]

for t in threads:
    t.start()

with Listener(on_press=on_press) as listener:
    listener.join()

stop_robot(SCRIPT_SOCKET)
SCRIPT_SOCKET.close()