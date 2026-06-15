import rtde.rtde as rtde
from math import sqrt

HOST = "172.17.0.2"
PORT = 30004

con = rtde.RTDE(HOST, PORT)
con.connect()
con.get_controller_version()

state_names = ("actual_TCP_pose",)
state_types = ("VECTOR6D",)
con.send_output_setup(state_names, state_types)
con.send_start()

prev = None 
start_move = None
moving = False
start_dist = 0
print("starting...")
for _ in range(10000):
    state = con.receive()
    if state is None:
        break
    tcp = state.actual_TCP_pose
    #print(tcp)
    x, y, z = tcp[0], tcp[1], tcp[2]
    if prev is not None:
        d = sqrt((x - prev[0])**2 + (y - prev[1])**2 + (z - prev[2])**2)
        #print(d)
        if d > 0.0001:
            if not moving: 
                print(" "*50,"\r",f"we are moving! ({d*1000:.1f} mm)", end="\r")
                start_move = prev
                moving = True
            else:
                start_dist = sqrt((x - start_move[0])**2 + (y - start_move[1])**2 + (z - start_move[2])**2)
                print(" "*50,"\r",f"{x:.3f} {y:.3f} {z:.3f}  ({start_dist*1000:.1f} mm)", end="\r")
        else:
            if moving: print(" "*50,"\r",f"{x:.3f} {y:.3f} {z:.3f}  ({start_dist*1000:.1f} mm)")
            print(" "*50,"\r","stoped.", end="\r")
            start_dist = 0
            start_move = None
            moving = False
    prev = (x, y, z)
print("done...")

con.send_pause()
con.disconnect()
