import threading
import queue
import time
from pynput import keyboard

# 1. Create a thread-safe queue to communicate between threads
input_queue = queue.Queue()

# 2. Define the keyboard listener callback functions
def on_press(key):
    try:
        # Put the alphanumeric key character into the queue
        input_queue.put(key.char)
    except AttributeError:
        # Handle special keys (like space, enter, esc)
        input_queue.put(key.name)

    # Stop listener if 'esc' is pressed
    if key == keyboard.Key.esc:
        return False

def start_keyboard_listener():
    """Function that starts the background keyboard listener."""
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()

# 3. Define the main program loop that gets influenced by the input
def main_program():
    print("Program started! Controls: 'a' (speed up), 's' (slow down), 'q' or 'esc' (quit)")
    
    current_speed = 1.0
    running = True
    
    while running:
        # Check if there are any new keyboard inputs in the queue without blocking
        try:
            while not input_queue.empty():
                user_input = input_queue.get_nowait()
                
                # Influence the program behavior based on input
                if user_input == 'a':
                    current_speed = max(0.1, current_speed - 0.2)
                    print(f"\n[Input 'a'] Speeding up! Loop interval: {current_speed:.1f}s")
                elif user_input == 's':
                    current_speed += 0.2
                    print(f"\n[Input 's'] Sowing down! Loop interval: {current_speed:.1f}s")
                elif user_input in ['q', 'esc']:
                    print("\nQuit signal received. Exiting...")
                    running = False
                    break
        except queue.Empty:
            pass

        # Simulate the program doing its regular work
        if running:
            print(".", end="", flush=True)
            time.sleep(current_speed)

# 4. Spin up the threads
if __name__ == "__main__":
    # Create and start the keyboard listener thread
    # Setting daemon=True ensures this thread dies when the main program dies
    listener_thread = threading.Thread(target=start_keyboard_listener, daemon=True)
    listener_thread.start()

    # Run the main program loop in the main thread
    main_program()