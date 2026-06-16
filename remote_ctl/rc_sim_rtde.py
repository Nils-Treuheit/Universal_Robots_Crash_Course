import socket
import rtde.rtde as rtde
from math import cos, sin, pi, sqrt
from pynput.keyboard import Listener, Key
from queue import Queue
from threading import Thread, Lock
from time import sleep

ROBOT_IP = "172.17.0.2"
RTDE_PORT = 30004
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

# ---------------------------------------------------------------------------
# Background Script Routine -> Full control over RTDE possible
# Script Interface is only used to start and stop Robot
# ---------------------------------------------------------------------------
def start_background_routine():
    prog = """
    running = True
    while running:
        changed = read_input_integer_register(0)
        sync()
        if changed == 1:
            changed = 0
            next_waypoint = p[read_input_float_register(0),
                              read_input_float_register(1),
                              read_input_float_register(2),
                              read_input_float_register(3),
                              read_input_float_register(4),
                              read_input_float_register(5)]
            sync()
            if running:
                movej(next_waypoint, a=0.5, v=0.3)
            end
            write_input_integer_register(0,0)
        end
        sync()
    end
    """
    s = socket.create_connection((ROBOT_IP, 30002))
    s.sendall(prog.encode() + b"\n")
    s.close()

def stop_robot():
    prog = """
    stopj(3.0)
    running = False
    """
    s = socket.create_connection((ROBOT_IP, 30002))
    s.sendall(prog.encode() + b"\n")
    s.close()

# TODO: Multi-Threaded RTDE program 
#
# - create a mother/movement planner thread 
#   - read current TCP position and joint speed
#   - read current target and write new target pos to input float register
#   - write to input integer register (use it as a changed target flag)
#   - send changes that the other threads publish
#     - follow the waypoint queue (if robot joint speeds are almost at 0 set target position to next waypoint)[Hints:
#       requires all reads and writes of registers - do not forget the integer changed flag;
#       only read from queue if joint_speed is almost zero]
#     - in edge case if offset is defined apply it to the waypoint before sending it as new target position and reset internal offset
#   - loop movement routines waypoints (original not manipulated target) in the queue but if halted event (press h) is triggered empty queue
#   - set internal helper falgs (e.g. robot_moving)
#   - start and stop robot with the two functions (start_background_routine(),stop_robot())
# - create a keyboard listener thread
#   - the arrow keys and +/- can be used to manipulate the robots position through the free movement thread (
#     arrow.left is -Y, arrow.right is +Y, arrow.up is -X and arrow.down is +X, "+" is +Z and "-"" is -Z )
#   - a shift of 2cm in the choosen direction should suffice
#   - if q is pressed the programm terminates
#   - if space bar is pressed the current movement is stopped/paused with stop_robot and only continued after pressing space again
#   - if h is pressed the current movement routine is stopped [stop_robot] and exited [empty waypoint_queue] (halt movement) 
# - create thread that schedules next way points based on choosen movement routine
#   - use Queue to put in waypoint schedule
#   - thread is only allowed to push in new waypoints if robot is standing still and Queue is empty
#   - we have circle, square, triangle and line (2 points) as shapes for the movement routine
#   - current TCP is the center point of the planned movement routine (center point of shape) 
#   - movements are planned on the y-z Plane (no movement along x)
# - create free movement thread to manipulate the target position of the robot at any given point in time
#   - coincides with the arrow keys and +/-
#   - if robot is not moving and waypoint queue is empty create a waypoint in the queue with the chosen position manipulation
#   - if robot is not moving but there are queued up way points and halted event (press h) is not triggered set program internal offset [Hint:
#     need to account for prior offset adding them together - attention offset is read and wrote to in many threads needs lock] 
#   - if robot is moving just change the target position directly (read input float register, 
#     manipulate values, write input float register, write 1 to input integer register)