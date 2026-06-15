If by "external control" you mean controlling a **Universal Robots (UR)** robot from a remote computer, the simplest approach is to send **URScript** over one of the robot's TCP interfaces.

The most commonly used ports are:

* **30001** — Primary interface
* **30002** — Secondary interface
* **30003** — Real-time interface (legacy)
* **30004** — RTDE (structured data exchange)

For a minimal example that remotely executes a motion, connect to port **30002** and send a URScript program.

### Python example

```python
import socket

ROBOT_IP = "192.168.1.100"  # change to your robot IP

script = """
def remote_move():
  movej([-1.57, -1.57, -1.57, -1.57, 1.57, 0.0], a=1.0, v=1.0)
end

remote_move()
"""

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((ROBOT_IP, 30002))
sock.sendall(script.encode("utf-8"))
sock.close()
```

When the script arrives, the controller compiles and executes it immediately.

### Even smaller example

```python
import socket

s = socket.create_connection(("192.168.1.100", 30002))

s.sendall(b"movej([-1.57,-1.57,-1.57,-1.57,1.57,0], a=1.0, v=1.0)\n")

s.close()
```

### Requirements

1. The robot must be powered on and not in an emergency stop state.
2. The robot must be in **Remote Control** mode (on e-Series robots).
3. Your PC must be able to reach the robot's IP address.
4. The target joint configuration must be valid and collision-free.

### More robust pattern

Many applications keep a long-running URScript program on the robot and communicate through:

* **RTDE** (recommended)
* TCP sockets created from URScript
* Registers and fieldbus interfaces

This is how the popular **ROS External Control URCap** works: a UR program runs on the robot and receives motion commands from an external PC.

If you're using the **External Control URCap** specifically (the one used with ROS/ROS2), the workflow is different from simply sending scripts to port 30002. In that case, tell me:

* Robot model (UR3e, UR5e, UR10e, etc.)
* PolyScope version
* Whether you're using the External Control URCap / ROS driver

and I can provide the minimum example for that setup.
