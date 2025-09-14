# UnderWater wireless communication

This project implements a **communication protocol between two microcontrollers (ESP8266/AVR/Arduino)** using **tone frequencies** over analog/digital pins.  

The system allows a **Master** device to send messages to a **Slave**, request data, and receive acknowledgments through **frequency-based signaling**.

---

## 🛠 Features
- Works as **Master** or **Slave** (toggle via `#define ROLE_MASTER` / `ROLE_SLAVE`).  
- Frequency-based protocol:  
  - Master sends @ **4 Hz**  
  - Slave acknowledges @ **2 Hz**  
- Encodes letters **A–Z** into frequencies stored in flash (`PROGMEM`) to save RAM.  
- Supports message sending (`SEND`) and receiving (`RECEIVE`).  
- Handles gaps, silence detection, and timeouts.  
- Cross-compatible with **AVR (Arduino Nano/Uno)** and **ESP8266**.  

---

## ⚙️ How It Works
- **Threshold Detection**:  
  Input signals on `A0` are compared against `ADC_THRESHOLD`.  
- **Tone Protocol**:  
  - Master sends a **4 Hz control tone** before transmitting data.  
  - Slave sends a **2 Hz control tone** as acknowledgment.  
- **Letter Encoding**:  
  - Each letter maps to a half-period stored in `letterDelay[]`.  
  - Messages are sent by toggling digital output at that frequency.  
- **Decoding**:  
  - Slave (or Master in reverse mode) samples analog input and matches measured frequencies to letters.

---

## 📍 Pin Connections

### Master
- **RX (input)** → `A0`  
- **TX (output)** → `D2` (ESP8266)  

### Slave
- **RX (input)** → `A0`  
- **TX (output)** → `D8` (Arduino Nano/Uno, etc.)  

*(Change pins in code if needed.)*

---

## 📦 Setup

1. Clone or copy the code to your Arduino project.  
2. In the source, select role:  

```cpp
#define ROLE_MASTER   // uncomment for Master
//#define ROLE_SLAVE   // uncomment for Slave

