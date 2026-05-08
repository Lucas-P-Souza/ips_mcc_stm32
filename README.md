# 🚀 DC Motor Control and Data Acquisition System (STM32 + Qt)

![STM32](https://img.shields.io/badge/STM32-CubeIDE-03234B?style=for-the-badge&logo=stmicroelectronics)
![Qt](https://img.shields.io/badge/Qt-C%2B%2B-41CD52?style=for-the-badge&logo=qt)
![Hardware](https://img.shields.io/badge/Hardware-In--The--Loop-FF4500?style=for-the-badge)

This repository contains the firmware (C) and graphical user interface (C++) developed for the real-time control and monitoring of a Direct Current (DC) Motor. The project uses an STM32 microcontroller interacting with a desktop application via Serial (UART) communication, allowing for precise analyses of the motor's dynamics ($\tau$ mechanical, current, speed ramp).

## ✨ Main Features (Infrastructure for *Asservissement*)

* **Signal Acquisition for Closed-Loop Control:**
  * **Dynamic Speed (A/B Channels):** Read via a hardware timer in quadrature at 100 Hz, ensuring the high resolution and stability required for future PID control.
  * **Absolute Validation (I Channel):** Read via external interrupt (EXTI) acting as a "source of truth" to confirm encoder pulses and ensure no accumulated error in the machine.
  * **Motor Current:** Read via ADC with the implementation of a Low-Pass Filter (smoothing) for torque monitoring and electrical safety.
* **Real-Time Telemetry:** Desktop interface developed in Qt with continuous plotting of the motor's response graphs (RPM and Current) using the `QCustomPlot` library.
* **Safety and Actuation:** Power drive via a 20 kHz PWM signal (highly efficient and inaudible) integrated with a hardware-isolated Emergency Brake system for workbench protection.

## 🛠️ Hardware Architecture

* **Controller Board:** NUCLEO-F411RE (ARM Cortex-M4)
* **Power Driver:** H-Bridge / *Hacheur* compatible with PWM signals.
* **Actuator/Sensors:** DC Motor coupled with an Optical/Magnetic Encoder (A, B, and I channels).
* **Current Sensor:** Analog module read via ADC. *(Note: A hardware voltage divider was used to protect the STM32's 3.3V ADC against 4V/5V spikes).*

### Main Pinout (STM32)
| Physical Pin | Microcontroller Function | Destination |
| :--- | :--- | :--- |
| **PA8 (D7)** | TIM1_CH1 (Advanced PWM) | Power Signal (Driver) |
| **PA6 / PA9** | GPIO Output | Direction of Rotation / Emergency Brake |
| **PB4 / PB5** | TIM3 (Encoder Mode TI12) | Encoder Channels A and B |
| **PB10 (D6)** | EXTI (External Interrupt) | Encoder Channel I (Index) |
| **PA0 (A0)** | ADC1 Channel 0 | Current Sensor |

---
*Développé dans le cadre du module d'Asservissement / Systèmes Commandés.*
**Professeur:** M. Bourgeot