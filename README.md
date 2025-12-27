# Circuit Raja – Hardware Hustle: Lam Research Challenge 2025

## Overview
This repository contains the complete system design and technical implementation for the **Hardware Hustle Round : Lam Research Challenge 2025**, developed by **Team Circuit Raja (Team ID: LRC-U-0053-6)**.

The solution consists of two coordinated robots and a centralized arena automation system designed to operate reliably under real-time constraints and complete all tasks within the competition time limit.

---

## System Architecture
The system follows a **Decentralized & Deterministic Control Philosophy** to eliminate latency bottlenecks and ensure predictable behavior.

### Major Subsystems
- **Advanced Line Follower Robot (ALFR)** – Autonomous navigation and safety
- **Single Arm Robot (SARM)** – Manual operation with automated manipulation
- **Arena Automation System** – Event-driven gate control
- **Custom Peristaltic Pump** – Precision fluid dispensing

---

## Advanced Line Follower Robot (ALFR)
- 8-channel analog line sensor array with **PD control**
- **Interrupt-driven ultrasonic obstacle detection** for instant emergency stops
- High-speed, stable navigation through sharp curves and S-paths
- Line re-acquisition using last-known turn memory
- Constrained motor control for safe and consistent behavior

---

## Single Arm Robot (SARM)
- **Dual-ESP32 decentralized architecture**
  - **Base Controller:** Mecanum wheel mobility and kinematics
  - **Arm Controller:** 5-DOF robotic arm and gripper control
- Omni-directional movement for precise obstacle handling
- Predefined arm **state macros** for fast, repeatable manipulation
- **Dual-pilot operation** using two independent Bluetooth Android apps
- Modified gripper with high-friction compliant padding for reliable pickup

---

## Arena Circuitry & Automation
- Centralized MCU with **non-blocking firmware design**
- **Gate 1:** Relay-controlled peristaltic pump dispensing exactly **125 mL**
- **Gate 2:** 5-second LED activation using `millis()` (non-blocking)
- **Gate 3:** Load cell (HX711) based weight verification
- TFT display showing team identity and real-time weight data

---

## Peristaltic Pump
- Custom 3D-printed positive displacement pump
- Three-roller design for continuous, backflow-free operation
- Flow rate derived analytically and calibrated experimentally
- Software-adjustable timing to maintain ±5% volume accuracy

---

## Key Design Highlights
- Deterministic PD control over heuristic methods
- Hardware interrupts for safety-critical sensing
- Parallel execution using multi-core and multi-operator strategies
- Fully non-blocking arena automation logic

---

## Team
**Circuit Raja**  
Hardware Hustle – Lam Research Challenge 2025  
Submission Date: **November 2025**
