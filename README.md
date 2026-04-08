# STM32 Microcontroller Project - Tasks 1, 2 & 3

This repository contains the implementation of the three basic tasks for the **IPS Module (Interface Puissance Système)** at ENIB, to be validated by M. Bourgeot.

## 🔌 Hardware Configuration

### Pinout Mapping
* **PA0 (A0):** Analog input (Potentiometer) for data acquisition.
* **PA8 (D7):** Digital output for the **10ms** periodic task.
* **PA9 (D8):** Digital output for the **1s** periodic task.
* **PA2 / PA3:** USART2 TX/RX for serial communication via USB.

### Timer Calculations (PSC & ARR)
Using the internal 84 MHz clock ($f_{clk}$):
$$f_{out} = \frac{f_{clk}}{(PSC + 1) \times (ARR + 1)}$$

* **TIM2 (1s Period):** PSC = 8399, ARR = 9999.
* **TIM3 (10ms Period):** PSC = 8399, ARR = 99.

[![App Platorm](https://wiki.st.com/stm32mcu/nsfr_img_auth.php/thumb/5/50/TIM_Img_16.png/1920px-TIM_Img_16.png)](https://wiki.st.com/stm32mcu/wiki/Getting_started_with_TIM)

---

## 🚀 Tasks Objectives

### Tâche 1: Periodic Task Programming
**Goal:** Program periodic tasks (1s and 10ms examples) and demonstrate the correct timing using an oscilloscope.
* **Implementation:** Configured **TIM2** and **TIM3** interrupts to toggle pins **PA9 (D8)** and **PA8 (D7)**.
    * Used the `HAL_TIM_PeriodElapsedCallback` for non-blocking execution.

### Tâche 2: Analog Data Acquisition
**Goal:** Acquire analog data from a potentiometer. Store the raw value in an `int` and the voltage in a `float`.
* **Implementation:** Configured **ADC1** on channel **PA0 (A0)** with 12-bit resolution.
    * Read the `uint32_t` raw value (0-4095) and converted it to a `float` (0.0V - 3.3V) in the main loop.

### Tâche 3: Serial Communication (UART)
**Goal:** Send data from the STM32 to the PC via UART over USB for visualization (e.g., Putty).
* **Implementation:** Configured **USART2** to transmit formatted strings containing both raw and voltage values.
    * Used `HAL_UART_Transmit` with `HAL_MAX_DELAY` to ensure data integrity.

## 💻 Environment
* **Board:** NUCLEO-F411RE
* **IDE:** STM32CubeIDE
* **Language:** C (HAL Library)
