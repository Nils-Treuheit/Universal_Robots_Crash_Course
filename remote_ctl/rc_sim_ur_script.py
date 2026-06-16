import socket
from time import sleep
from copy import deepcopy

ROBOT_IP = "172.17.0.2"
ROBOT_SCRIPT_PORT = 30002

def explicit_long():
  #print(" " * 50, "\r", "Execute explicit_long!", end="\r")
  script = """
  def remote_move():
    movej([-1.57, -1.57, -1.57, -1.57, 1.57, 0.0], a=1.0, v=1.0)
  end

  remote_move()
  """
  sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  sock.connect((ROBOT_IP, ROBOT_SCRIPT_PORT))
  sock.sendall(script.encode("utf-8"))
  sock.close()


def implicit_short():
  #print(" " * 50, "\r", "Execute implicit_short!", end="\r")
  s = socket.create_connection((ROBOT_IP, ROBOT_SCRIPT_PORT))
  s.sendall(b"movej(p[-0.214,-0.012,0.676,0,-3.140795,0], a=1.0, v=1.0)\n")
  s.close()


def draw_square(keyboard_ctl=None, stop_event=None, prev = None):
  print(" " * 50, "\r", "Execute drawing square!", end="\r")
  wp1 = [0.24, -0.2, 0.644, 1.2092, 1.2092, 1.2092]
  wp2 = [0.24, -0.2, 0.422, 1.2092, 1.2092, 1.2092]
  wp3 = [0.24, -0, 0.422, 1.2092, 1.2092, 1.2092]
  wp4 = [0.24, -0, 0.644, 1.2092, 1.2092, 1.2092]
  square_points = [wp1,wp2,wp3,wp4]

  s = socket.create_connection((ROBOT_IP, ROBOT_SCRIPT_PORT))
  did_stop = False
  prev = None
  for wp in square_points:
    if keyboard_ctl and not keyboard_ctl.is_alive(): break
    if stop_event and stop_event.is_set():
      did_stop = True
      while(stop_event.is_set()): continue
    if keyboard_ctl and not keyboard_ctl.is_alive(): break
    if did_stop and prev:
      print("move to prior pos!")
      s.sendall(f"movej(p{str(prev)}, a=0.5, v=0.5)\n".encode('utf-8'))
      did_stop = False
      sleep(2.5)
    print("move to current pos!")
    s.sendall(f"movej(p{str(wp)}, a=0.5, v=0.5)\n".encode('utf-8'))
    prev = deepcopy(wp)
    sleep(2.5)
  s.close()
  return prev


if __name__=="__main__":
  from pynput.keyboard import Listener, Key, KeyCode
  from threading import Thread, Event
  from queue import Queue
  input_queue = Queue()
  stop_flag = Event()

  def on_press(key):
    if isinstance(key, KeyCode): input_queue.put(key.char)
    else: input_queue.put(key.name)
    print(" " * 50, "\r", "Pressed", key, end="\r")
    if (isinstance(key, KeyCode) and key.char in ['c', 'q']) or key in [Key.esc, Key.end]:
      if stop_flag.is_set(): input_queue.put(Key.space.name)
      return False

  def listener_wrapper():
    with Listener(on_press=on_press) as listener:
      listener.join()


  def interrupt(kl):
    s = socket.create_connection((ROBOT_IP, ROBOT_SCRIPT_PORT))
    while kl.is_alive():
      if not input_queue.empty() and input_queue.get_nowait() == Key.space.name:
        stop_flag.set()
        s.sendall(b"stopj(3.0)\n")
        print("Interrupted movement!")
        sleep(0.05)
        while input_queue.get() != Key.space.name: continue
        stop_flag.clear()
    s.close()

  key_listener = Thread(target=listener_wrapper, daemon=True)
  key_listener.start()
  print("Keyboard Listener running:", key_listener.is_alive())

  interruptor = Thread(target=interrupt, args=(key_listener,), daemon=True)
  interruptor.start()
  print("Interruptor Thread running:", interruptor.is_alive())

  print("Interruptor active:", stop_flag.is_set())
  print("Keyboard Queue is empty:", input_queue.empty())

  print("Starting two point loop ...")
  first_it = True
  current_move = None
  next_move = explicit_long
  following_is_explicit = False
  while key_listener.is_alive():
    did_stop = False
    if not first_it: sleep(3)
    if stop_flag.is_set():
      did_stop = True
      print("Stopped while executing", ("implicit_short!" if following_is_explicit else "explicit_long!"))
      while stop_flag.is_set(): sleep(0.2)
    if not key_listener.is_alive(): break
    if did_stop and (current_move is not None): current_move()
    else:
      next_move()
      current_move = next_move
      next_move = explicit_long if following_is_explicit else implicit_short
      following_is_explicit = not(following_is_explicit)
      first_it = False
  print("... Finished two point loop")
  sleep(3)

  # restart the Threads
  key_listener = Thread(target=listener_wrapper, daemon=True)
  key_listener.start()
  print("Keyboard Listener running:", key_listener.is_alive())

  interruptor = Thread(target=interrupt, args=(key_listener,), daemon=True)
  interruptor.start()
  print("Interruptor Thread running:", interruptor.is_alive())

  print("Interruptor active:", stop_flag.is_set())
  print("Keyboard Queue is empty:", input_queue.empty())

  print("Starting square drawing loop ...")
  prev = None
  while key_listener.is_alive():
    prev = draw_square(key_listener, stop_flag, prev)
  print("... Finished square drawing loop")
  key_listener.join()
  interruptor.join()
  from sys import exit
  exit()
