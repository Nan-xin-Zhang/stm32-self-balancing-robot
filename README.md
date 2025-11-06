# STM32 Self-Balancing Car (MPU6050 + PID Control)

This project implements a **two-wheel self-balancing robot** based on the **STM32F103C8T6** microcontroller.  
The system reads attitude data from the **MPU6050 IMU**, performs **dual-loop PID control**, and drives DC motors through **PWM** to maintain dynamic balance.

---

## 🧠 System Overview
**Core idea:** Use an IMU to detect the tilt angle, run PID algorithms to calculate correction torque, and drive both wheels to keep the robot upright in real time.

**Main functions**
- Real-time tilt detection using MPU6050 accelerometer + gyroscope  
- Complementary filter for angle estimation  
- Dual-loop PID control (angle + speed loop)  
- PWM motor driving for wheel motion  
- UART debugging output for PID tuning  

---

## ⚙️ Hardware Platform

| Component | Description |
|------------|-------------|
| MCU | STM32F103C8T6 (ARM Cortex-M3) |
| IMU | MPU6050 (6-axis accelerometer + gyro) |
| Motor Driver | L298N / TB6612 (dual H-bridge) |
| Motors | DC geared motors with encoders |
| Power | 7.4 V Li-Po battery |
| Optional Modules | OLED display (I²C) for data output, Bluetooth HC-05 for remote control |

---

## 🧩 Software Architecture

```text
balance-car/
├── user/                 # Main application: PID, control loops, main.c
├── my_lib/               # Drivers and reusable modules (PID, I2C, OLED, delay, etc.)
├── std_periph_driver/    # STM32 official peripheral library
├── startup/              # MCU startup assembly file
├── doc/                  # Schematics, notes, and reference PDFs
└── balance_car.uvprojx   # Keil uVision project file




---

## 🔬 Control Algorithm

- **Sensor Fusion:** Complementary filter combining accelerometer and gyro data.  
- **Dual PID Control:**
  - **Inner Loop (Angle PID)** — keeps the car upright by adjusting wheel torque.
  - **Outer Loop (Speed PID)** — maintains desired velocity and suppresses drift.
- **PWM Motor Drive:** converts PID output to wheel speed and direction.

---

## 📊 Key Source Files

| File | Description |
|------|--------------|
| `user/app_control.c` | Implements dual-loop PID logic |
| `user/app_motor.c` | PWM motor control and direction switching |
| `user/app_encoder.c` | Reads encoder feedback |
| `my_lib/mpu6050.c` | IMU communication and angle calculation |
| `my_lib/pid.c` | Generic PID controller |
| `my_lib/delay.c` | Delay and timing utilities |

---

## 🧪 Features Completed
- [x] MPU6050 initialization and I²C communication  
- [x] Angle estimation via complementary filter  
- [x] PID control implementation  
- [x] PWM motor speed control  
- [x] Encoder feedback reading  
- [ ] Bluetooth control interface *(planned)*  
- [ ] OLED display output *(planned)*  

---

## 🧰 Development Environment
- **Toolchain:** Keil uVision 5  
- **Language:** Embedded C  
- **Debug:** UART serial terminal + logic analyzer  
- **OS:** Windows / Linux / macOS  

---

## 🚀 How to Build & Run
1. Open `balance_car.uvprojx` in **Keil uVision 5**  
2. Compile the project  
3. Connect STM32 board via **ST-Link**  
4. Download the program to the board  
5. Tune PID parameters in `pid.c` to achieve stable balance  

---

## 📈 Future Improvements
- Kalman filter for smoother angle estimation  
- Adaptive PID auto-tuning  
- Bluetooth remote control and mobile app  
- OLED real-time display of angle and speed  
- 3D simulation in MATLAB/Simulink  

---

## 🧾 License
MIT License — free for educational and personal use.

---

## 👤 Author
**Nanxin Zhang**  
M.Sc. student in Electronics Engineering, Linköping University  
📍 Linköping, Sweden  
📧 znanxin5@gmail.com
