# Prototype Noise Source Direction Detection Based on Threshold Value and Time Difference of Arrival (TDOA) Using Internet of Things (IoT)
## Overview

This project is a prototype system for detecting and determining the direction of a noise source using the Time Difference of Arrival (TDOA) method and Internet of Things (IoT) technology.

The system utilizes four GY-MAX4466 microphone sensors arranged in a cross-array configuration and an ESP32 microcontroller to identify the direction of incoming sound. The detection results are transmitted through the MQTT protocol and visualized on a ThingsBoard dashboard for real-time monitoring.

## System Architecture
1. Microphones capture incoming sound signals.
2. ESP32 records the arrival time of the sound wave at each microphone.
3. TDOA algorithm calculates time differences between microphone pairs.
4. Direction and azimuth angle are estimated using atan2(x, y) trigonometric calculations.
5. Results are transmitted to ThingsBoard via MQTT.
6. Dashboard displays real-time monitoring data.

### Applications
This prototype can be applied to:
- Noise monitoring systems
- Smart classrooms
- Smart libraries
- Smart offices
- Environmental monitoring
- Security and surveillance systems
- Sound localization research
- IoT-based monitoring platforms

### Limitations
- Supports only a single dominant sound source.
- Direction estimation accuracy is affected by environmental noise, reflections, atc.
- Does not calculate the exact position of the sound source.
- Designed as a prototype and proof-of-concept implementation.

*dionp3, 2026*
