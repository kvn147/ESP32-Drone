# ESP32-Drone

### Objective
Design and build a fully open-source quadcopter drone from scratch using an ESP32-WROOM microcontroller.

## Getting Start
**Prequisites**
- ESP32 dev board
- MPU6050 module
- Up-to-date ESP-IDF framework (v5.4+)
- ESP-IDF extension

**Clone and build project**
```bash
git clone https://github.com/kvn147/ESP32-Drone.git
idf.py build
``` 

### Project Scope
- Mechanical: 3D-printed frame for drone.
- Electrical Hardware: ESP32 and MPU6050 to develop the flight controller and firmware. ESC for speed control. LEDs for error detection.
- Firmware: Sensor calibration, attitude estimation, PID control, motor mixing (pilot input translated to flight behavior).

### Progress (so far)
- [x] Research Articles and Videos.
- [x] Gather Electronic Components (ESP32, MPU6050, LEDS).
- [x] Set up GitHub & ESP IDF Project.
- [ ] Interface MPU6050 with microcontroller.
- [ ] 3D Print Drone Quad Frame.

### Resources
- Electronics: ESP32-WROOM, MPU6050, ESC, Motors, Propellers, Batteries, LEDS.
- Tools: Bambu Lab P1S 3D Printer, Soldering, RC transmitter/receiver (?).
- Software: VS Code, ESP-IDF Extension, Fusion 360, KiCad (?), GitHub.

#### Meeting Agenda (06/25):
1. Gathering external components.
2. Decide on 3D printed frame.
3. Setup GitHub repo.

#### Future Meetings TBD