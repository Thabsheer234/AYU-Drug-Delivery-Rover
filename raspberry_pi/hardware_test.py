import serial
import threading
import time
import sys

BAUD_RATE = 115200
POTENTIAL_PORTS = ['/dev/serial0', '/dev/ttyAMA0', '/dev/ttyS0']

# --- 1. ROBUST SERIAL INITIALIZATION ---
esp = None
active_port = None

for port in POTENTIAL_PORTS:
    try:
        s = serial.Serial(port, BAUD_RATE, timeout=0.5)
        time.sleep(0.3)
        s.reset_input_buffer()
        esp = s
        active_port = port
        print(f"[CONNECTED] Serial open on {active_port} at {BAUD_RATE} baud.")
        break
    except Exception:
        continue

if not esp:
    print("[ERROR] Could not open any serial port (/dev/serial0, /dev/ttyAMA0, /dev/ttyS0).")
    print("Check: sudo raspi-config -> Interface Options -> Serial Port (Login Shell: No, Hardware: Yes)")
    sys.exit(1)

# --- 2. ASYNC ESP32 LISTENER THREAD ---
running = True
def listen_esp32():
    while running:
        try:
            if esp and esp.in_waiting > 0:
                line = esp.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    # Highlight safety reflex alerts
                    if "OBSTACLE" in line:
                        print(f"\n⚠️  [ESP32 ALERT]: \033[91m{line}\033[0m")
                    else:
                        print(f"\n📥 [ESP32]: \033[92m{line}\033[0m")
                    print("\nCommand >> ", end="", flush=True)
        except Exception:
            break
        time.sleep(0.02)

listener = threading.Thread(target=listen_esp32, daemon=True)
listener.start()

def send(cmd):
    cmd_clean = cmd.strip()
    if not cmd_clean:
        return
    print(f"📤 [SENDING]: {cmd_clean}")
    esp.write(f"{cmd_clean}\n".encode('utf-8'))
    time.sleep(0.05)

# --- 3. INTERACTIVE CLI INTERFACE ---
def print_help():
    print("""
============================================================
              🤖 ROBOT HARDWARE CLI TESTER
============================================================
 [DRIVE CONTROLS]
   f          : Move Forward (Autostops if obstacle < 15cm)
   b          : Move Backward
   l          : Continuous Spin Left
   r          : Continuous Spin Right
   s or space : STOP all motors immediately

 [MICRO-PULSE ALIGNMENT (Testing Visual Centering)]
   pl         : Micro-pulse Left  (40ms)
   pr         : Micro-pulse Right (40ms)
   pf         : Micro-pulse Forward
   pb         : Micro-pulse Backward

 [TURN RADIUS TUNING]
   t90        : Test 90° Turn Right (T:90)
   t-90       : Test 90° Turn Left  (T:-90)
   t180       : Test 180° Turn      (T:180)
   ms <value> : Calibrate 90° duration in ms (e.g. 'ms 600')

 [PERIPHERALS]
   dist       : Read live ultrasonic distance (cm)
   open       : Open medicine lid servo (90°)
   close      : Close medicine lid servo (0°)
   beep       : Sound buzzer for 200ms
   loop_dist  : Continuously stream distance for 10 seconds

 [SYSTEM]
   h or ?     : Print this command menu
   raw <cmd>  : Send any raw string directly to ESP32
   q or quit  : Stop robot and exit
============================================================
""")

print_help()

try:
    while True:
        user_input = input("Command >> ").strip().lower()
        if not user_input:
            continue

        # Exit
        if user_input in ["q", "quit", "exit"]:
            send("S")
            break

        # Help
        elif user_input in ["h", "help", "?"]:
            print_help()

        # Motor Driving
        elif user_input in ["f", "forward"]:
            send("F")
        elif user_input in ["b", "back", "backward"]:
            send("B")
        elif user_input in ["l", "left"]:
            send("L")
        elif user_input in ["r", "right"]:
            send("R")
        elif user_input in ["s", "stop", " "]:
            send("S")

        # Alignment Micro-pulses
        elif user_input == "pl":
            send("PULSE_L")
        elif user_input == "pr":
            send("PULSE_R")
        elif user_input == "pf":
            send("PULSE_F")
        elif user_input == "pb":
            send("PULSE_B")

        # Turn testing & radius calibration
        elif user_input == "t90":
            send("T:90")
        elif user_input in ["t-90", "t270"]:
            send("T:-90")
        elif user_input == "t180":
            send("T:180")
        elif user_input.startswith("ms "):
            try:
                ms_val = int(user_input.split()[1])
                send(f"SET_TURN_MS:{ms_val}")
                print(f"[SETTING] Turn 90° timing updated to {ms_val} ms.")
            except ValueError:
                print("[ERROR] Usage: ms <milliseconds>, e.g., 'ms 580'")

        # Ultrasonic Sensor
        elif user_input in ["dist", "d"]:
            send("DIST")
        elif user_input == "loop_dist":
            print("[INFO] Streaming distance for 10 seconds (Ctrl+C to cancel)...")
            try:
                for _ in range(20):
                    send("DIST")
                    time.sleep(0.5)
            except KeyboardInterrupt:
                print("\n[STREAM ENDED]")

        # Servo & Buzzer
        elif user_input == "open":
            send("OPEN")
        elif user_input == "close":
            send("CLOSE")
        elif user_input in ["beep", "p"]:
            send("BEEP")

        # Send raw custom command (e.g. "raw T:45")
        elif user_input.startswith("raw "):
            raw_payload = user_input[4:].strip()
            send(raw_payload)

        else:
            print(f"Unknown command: '{user_input}'. Type 'h' for list of commands.")

except KeyboardInterrupt:
    print("\nInterrupted! Shutting down...")
    send("S")

finally:
    running = False
    send("S")
    if esp and esp.is_open:
        esp.close()
    print("[DISCONNECTED] Port closed.")