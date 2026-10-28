# 🤖 AYU — Drug Delivery Rover

An autonomous ground delivery robot powered by a **Raspberry Pi Zero W** (high-level vision, BFS pathfinding, and Flask web dashboard) and an **ESP32** (real-time motor control, ultrasonic reflex braking, and compartment actuation) communicating over UART.

## 🧠 System Architecture
* **High-Level Brain (Raspberry Pi Zero W):**
  * Captures floor navigation frames via USB webcam (`V4L2` + CLAHE contrast normalization).
  * Decodes floor QR junctions and performs closed-loop visual micro-centering.
  * Computes shortest delivery routes dynamically using **Breadth-First Search (BFS)**.
  * Hosts a real-time web control panel and live MJPEG inspection stream.
* **Low-Level Controller (ESP32):**
  * Drives dual DC gearmotors via a DFRobot `DIR`/`PWM` motor driver.
  * Runs a non-blocking `< 15 cm` ultrasonic obstacle interrupt loop (auto-pause, continuous buzzer alarm, and auto-resume when clear).
  * Actuates an SG90 servo to unlock the delivery compartment for 20 seconds upon arrival.

## 🔌 Hardware Pinout (ESP32 ↔ Peripherals)

| Component | Component Pin | ESP32 Pin | Notes |
| :--- | :--- | :--- | :--- |
| **Raspberry Pi Zero** | `TXD` (Pin 8) / `RXD` (Pin 10) | `RX2` (GPIO 16) / `TX2` (GPIO 17) | 115200 baud UART |
| **DFRobot Motor Driver** | `DIR1`, `PWM1`, `PWM2`, `DIR2` | `D25`, `D26`, `D27`, `D14` | Left & Right Motors |
| **HC-SR04 Ultrasonic** | `TRIG`, `ECHO` | `D5`, `D18` | `< 15cm` auto-stop reflex |
| **SG90 Servo** | Signal (Orange) | `D13` | Compartment lid lock |
| **Piezo Buzzer** | Positive (`+`) | `D19` | Obstacle & arrival alert |

## 🚀 How to Run

### 1. Flash ESP32 Firmware
Open `firmware/bot_esp32/bot_esp32.ino` in the Arduino IDE, select **ESP32 Dev Module**, and upload.

### 2. Run Raspberry Pi Controller
```bash
cd raspberry_pi
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
python3 app.py
```

Open http://<raspberry-pi-ip>:5000 in your browser.