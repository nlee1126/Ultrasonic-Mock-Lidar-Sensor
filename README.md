# Ultrasonic “Mock LiDAR”

Uses a **GY-521 (MPU6050)** + **HC-SR04 ultrasonic sensor** on a **servo** to scan and plot yaw & distance in real time.

## Hardware

* Arduino Uno/Nano (or compatible)
* GY-521 (MPU6050), HC-SR04, Servo, Push Button
* Mini breadboard + jumpers

## Wiring (default in `.ino`)

* **GY-521**: VCC→3.3/5 V, GND→GND, SDA→A4, SCL→A5
* **HC-SR04**: VCC→5 V, GND→GND, TRIG/ECHO→digital pins
* **Servo**: SIG→PWM pin, VCC→5 V, GND→GND
* **Button**: Pin→digital, other→GND (INPUT\_PULLUP)

## Run

1. Mount GY-521 + HC-SR04 on servo horn.
2. Calibrate yaw offset in `.ino`.
3. Upload `arduino_script_w_servo.ino` (or `_wo_servo.ino`).
4. Press button to scan.
5. `pip install pyserial numpy matplotlib`
6. Edit `plotter.py` for your port/baud, then run:
