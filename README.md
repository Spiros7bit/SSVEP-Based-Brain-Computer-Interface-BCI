# SSVEP-Based Brain-Computer Interface

## Overview

This project implements a real-time Steady-State Visually Evoked Potential (SSVEP) Brain-Computer Interface (BCI) using an ESP32 microcontroller and a custom PCB for analog interface. It was developed as part of a thesis and demonstrates how EEG signals can be processed and classified in real time on embedded hardware.

The system detects brain responses to visual stimuli at specific frequencies and converts them into control decisions.

---

## System Architecture

The signal processing pipeline consists of the following stages:

1. **Analog Interface**
   
   * Instrumentation Amplifier
   * Low Pass Filter, Sallen Key, 2 order
   * AC amplifier
   * Notch 50 Hz filter
   * Final AC Amplifier
   * Right Leg Driver circuit
     
3. **Signal Acquisition**

   * ADC sampling at 1 kHz
   * Hardware timer interrupt for precise timing

4. **Preprocessing**

   * DC offset removal (IIR high-pass filter)
   * FIR band-pass filtering

5. **Feature Extraction**

   * Goertzel algorithm for frequency detection
   * Target frequencies: 10 Hz, 12 Hz, 20 Hz, 24 Hz

6. **Decision Making**

   * Argmax power comparison
   * Median Absolute Deviation (MAD)-based classifier

7. **Output**

   * Real-time signal visualization via Serial Plotter
   * Classification results (Argmax & MAD)

---

## Features

* Analog interface and Acquition System
* Real-time processing on ESP32
* 1 kHz sampling using hardware timer interrupts
* Efficient fixed-point DSP implementation
* FIR filtering with circular buffer
* Sliding window Goertzel analysis (with overlap)
* Robust classification using MAD
* Serial output for debugging and visualization

---

## Hardware Requirements

* ESP32 development board
* EEG acquisition circuit (as designed in the thesis)
* Electrodes and analog front-end

**ADC Input:** GPIO34 (ADC1_CHANNEL_6)

---

## Software Requirements

* Arduino IDE or PlatformIO
* ESP32 board support package

---

## How It Works

### Sampling

A hardware timer triggers ADC sampling at 1000 Hz. Each sample is processed in real time.

### DC Removal

A simple IIR filter removes the DC component from the signal:

```
dc_est = dc_est + k * (x - dc_est)
```

### FIR Filtering

A band-pass FIR filter is applied to isolate relevant EEG frequencies.

### Goertzel Algorithm

The Goertzel algorithm is used to compute the energy at specific target frequencies efficiently.

* Window size: 2048 samples
* Step size: 256 samples (overlap processing)

### Decision Logic

Two decision methods are implemented:

1. **Argmax**

   * Compares power between frequency groups

2. **MAD (Median Absolute Deviation)**

   * Computes a robust threshold based on signal statistics
   * Reduces noise and false detections

---

## Output Format

The system outputs data via Serial in the following format:

```
signal_value, argmax_decision, mad_decision
```

This can be visualized using the Arduino Serial Plotter.

---

## Project Structure

```
.
├── main.ino
├── fir_new_2_1000hz.h   # FIR filter coefficients
├── README.md
```

---

## Applications

* Brain-computer interfaces (BCI)
* Assistive technologies
* Neuroengineering research
* Real-time embedded DSP systems

---

## Future Improvements

* Add more target frequencies
* Improve classification accuracy
* Wireless data transmission (Bluetooth/WiFi)
* GUI for real-time monitoring
* Add a TinyML algorithm

---

## Author

Developed as part of a thesis project on SSVEP-based Brain-Computer Interfaces.
