# CPE-312_DHT-11

## 📖 Overview
This project was created as part of **CPE-312: Embedded System Laboratory**.

Building upon the previous CPE-214 project, this version adds **closed-loop actuation**: a DC motor (e.g., a fan) is driven with **PWM** based on the measured temperature from a **DHT11** sensor. Readings are shown on the on-board glass LCD, and a user button toggles the display between temperature and humidity. An on-board LED indicates when the system is actively cooling.

---

## ✨ Features
- **DHT11 sensor** integration using precise timing (DWT-based microsecond delays).
- **Live display** on STM32L152 on-board glass LCD:
  - Toggle **Temperature (°C)** / **Humidity (%)** with the user button (EXTI).
- **Temperature-controlled motor drive**:
  - **PWM on TIM4 CH2** (PB7) controls motor speed.
  - **Enable** (PA4) gates motor power; **direction** pins set “Right” rotation.
  - Default logic: if **temperature > 28 °C**, motor is enabled and duty is computed as  
    `duty = 35 + 5 * (temperature − 28)` (clamped by timer’s 0–100 ARR window).
- **Visual indicator**: on-board LED (PB6) turns ON when the motor is active.
- **Interrupt-driven UI** via **EXTI0** (PA0).
- **Low-Layer (LL) drivers** for tight, efficient control.

---

## 🧩 Flowchart
(added later)

---

## ▶️ Example Workflow
1. **Init**  
   Configure system clock (32 MHz), LCD, GPIO, EXTI0, DWT delay, and TIM4 PWM.
2. **Direction setup**  
   Set motor direction to “Right” (H-side/PWM path enabled, L-side held low).
3. **Sense**  
   Read DHT11 and parse 40-bit frame → `temperature`, `humidity`.
4. **Decide**  
   If `temperature > 28 °C` → compute duty and:
   - Enable motor (PA4 = HIGH)
   - Update TIM4 CH2 compare value (PB7 PWM)
   - Turn ON LED (PB6 = HIGH)  
   Else → disable motor and turn OFF LED.
5. **Display**  
   Show `T xx` or `H xx` on the LCD; press button (PA0) to toggle the view.
6. **Loop**  
   Repeat sense → decide → actuate → display.

---

## 🚀 Future Improvements
- **Configurable thresholds** and duty mapping via button UI or serial console.
- **Humidity-aware policy** (e.g., combine T and RH for comfort index control).
- **Smoother PWM** (raise PWM frequency by adjusting PSC/ARR to reduce audible buzz).
- **Safety & telemetry**: add stall detection, overcurrent flag, UART logs, or EEPROM/Flash logging.
- **Driver abstraction**: replace direct LL calls with a small HAL for portability.

---

## 🛠️ Hardware Used
- **STM32L152 Discovery Board** (with on-board glass LCD)
- **DHT11** Temperature/Humidity sensor (with an external pull-up on DATA recommended)
- **DC Motor + driver** (e.g., **L293D**/compatible H-bridge; the code names this block `_1293d`)
- **On-board LED** (PB6)
- **User button** (PA0)

---

## 📍 Pin Allocation

