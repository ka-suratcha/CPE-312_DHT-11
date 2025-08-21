# CPE-312_DHT-11

## 📖 Overview
This project was created as part of **CPE-312: Embedded System Laboratory**.

Building upon the previous CPE-214 project, this version upgrades the system from a simple “alarm-only” design to temperature-driven actuation. In CPE-214, the device read the DHT11, showed the value on the LCD, and when the **temperature exceeded** a fixed threshold it just turned on LEDs and a **DAC-driven buzzer** (T > 20 °C) and displayed either temperature or humidity with a button toggle. In CPE-312, the device still measures and displays T/RH, but now it drives a **DC motor (fan) with PWM** when the temperature is high (T > 28 °C): it computes a duty cycle and enables the motor, turning an LED on as an activity indicator.


### 🔁 What changed from CPE-214 → CPE-312 
- **Control behavior**
  - **CPE-214:** Alarm-only — if `T > 20 °C`, turn **LEDs + DAC buzzer** on; otherwise off.
  - **CPE-312:** Active cooling — if `T > 28 °C`, compute a PWM duty and **drive a motor (fan)**; LED indicates motor active.

- **Actuators / Outputs**
  - **CPE-214:** DAC on **PA5** for the buzzer + LEDs (**PA11**, **PB6**).
  - **CPE-312:** PWM on **TIM4 CH2 (PB7)** to a motor driver, **ENABLE on PA4**, **DIR on PA11**; **PB6** LED used as an “active cooling” indicator.

- **Sensor timing**
  - **CPE-214:** Bit timing via simple busy-wait loops and counters.
  - **CPE-312:** Microsecond-accurate **DWT delays** for more robust DHT11 reads.

- **Threshold policy**
  - **CPE-214:** Fixed alarm threshold at **20 °C** (binary on/off).
  - **CPE-312:** Higher threshold at **28 °C** with **proportional PWM** (duty scales with temperature above the threshold).

- **User interface**
  - **Both:** **PA0** button toggles LCD view between temperature and humidity; values are shown on the **STM32L152 glass LCD**.
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
1. **System Initialization**:  
   Configure system clock (32 MHz), LCD, GPIO, EXTI0, DWT delay, and TIM4 PWM.
2. **Direction setup**  
   Set motor direction to “Right” (H-side/PWM path enabled, L-side held low).
3. **DHT11 Data Acquisition**:
   - Continuously read 40-bit data frame from DHT11.
   - Extract `humidity` and `temperature` values.
4. **Condition Check**: 
   If `temperature > 28 °C`→
   - Enable motor (PA4 = HIGH)
   - Update TIM4 CH2 compare value (PB7 PWM)
   - Turn ON LED (PB6 = HIGH)  
   Else → disable motor and turn OFF LED.
5. **Display on LCD**:
   - Default display: Temperature.  
   - Press **user button (PA0)** → switch to humidity view.  
   - Press again → switch back to temperature.

---

## 🚀 Future Improvements
- **Configurable thresholds** and duty mapping via button UI or serial console.
- **Smoother PWM** (raise PWM frequency by adjusting PSC/ARR to reduce audible buzz).
- Store historical data in **EEPROM/Flash** for later analysis.
- Add **serial output (UART)** for data logging on PC.
- Support additional sensors (e.g., DHT22 for better accuracy).
- Add **humidity warnings** to the list of buzzer alerts.

---

## 🛠️ Hardware Used
- **STM32L152 Discovery Board** (with on-board glass LCD)
- **DHT11** Temperature/Humidity sensor (with an external pull-up on DATA recommended)
- **DC Motor + driver** (e.g., **L293D**/compatible H-bridge; the code names this block `_1293d`)
- **On-board LED** (PB6)
- **User button** (PA0)

### 📍 Pin Allocation
<img width="2155" height="666" alt="image" src="https://github.com/user-attachments/assets/9409cd06-742d-4608-9f36-382de4b3115e" />


### 📸 Real Hardware Setup

