# Hermes' Hand 🖐️: Real-Time Sign Language Recognition Glove

![C](https://img.shields.io/badge/C-00599C?style=for-the-badge&logo=c&logoColor=white)
![STM32](https://img.shields.io/badge/STM32-03234B?style=for-the-badge&logo=stmicroelectronics&logoColor=white)
![RTOS](https://img.shields.io/badge/ChibiOS-RTOS-orange?style=for-the-badge)
![Embedded Systems](https://img.shields.io/badge/Embedded_Systems-000000?style=for-the-badge)

**Hermes' Hand** is a wearable, sensorized glove designed to translate Sign Language into text in real-time. Developed during the **Neapolis Summer Campus at STMicroelectronics** and officially exhibited at **Maker Faire Rome**, this project bridges the communication gap for the deaf community using low-power edge computing. 

## 🚀 Key Features
* **Real-Time Translation**: Translates both static signs (rule-based) and dynamic gestures (timed state machine) with minimal latency.
* **Wireless Communication**: Transmits translated text wirelessly to any external host (PC/Smartphone) via a TCP/IP socket.
* **RTOS Integration**: Utilizes ChibiOS for efficient multithreading, ensuring stable data acquisition and processing.
* **Lightweight & Cost-Effective**: Built using accessible hardware without relying on heavy, computationally expensive external cameras.

## 💻 Hardware Architecture
The physical architecture is divided into three interconnected modules:
* **Processing Unit**: STM32 Nucleo-L432KC (ARM Cortex-M4), operating at 80 MHz for fast digital signal processing.
* **Sensor Module**: 
  * 5x **Spectra Symbol 2.2" Flex Sensors** mapped to a 12-bit ADC via voltage dividers to measure finger flexion.
  * **ISM330DLC 6-axis IMU** (accelerometer & gyroscope) communicating via I2C to determine the hand's spatial orientation and trajectory.
* **Wi-Fi Module**: Espressif ESP8266EX SoC, communicating with the STM32 via UART to broadcast the translated text strings.

## ⚙️ Software & Algorithms
The firmware is written in C and runs on **ChibiOS**, leveraging memory alignment and parallel threads (main thread and capture thread). 
* **Data Processing**: Raw analog data from the flex sensors is converted into a 0-100% flexion scale. The IMU calculates gravitational acceleration to classify hand orientation (e.g., Palm Up, Portrait).
* **Classification**: 
  * **Static Signs**: A rule-based algorithm compares the quantized finger mapping and hand orientation against a predefined struct table (e.g., translating the ASL letter 'B').
  * **Dynamic Signs**: A two-stage sequential state machine analyzes transitional movements within specific time windows (e.g., translating 'Thank You').

## 🔮 Future Work
The prototype lays the groundwork for advanced wearable iterations. Planned improvements include:
* **Deep Learning Integration**: Replacing the rule-based algorithm with sequence models (LSTM or GRU) to automate feature extraction and expand the vocabulary.
* **Hardware Miniaturization**: Designing a custom PCB to eliminate wires and position all electronics directly on the back of the hand.
* **Mobile Application**: Developing a dedicated smartphone app utilizing text-to-speech APIs to vocalize translations instantly.
* **Dual-Hand Tracking**: Integrating a second mirrored glove for the non-dominant hand.

## 📄 Documentation
For an in-depth look at the mathematical models, hardware selection, and state machine logic, please read the full [Bachelor's Thesis (English) PDF](./docs/Thesis_eng.pdf).
