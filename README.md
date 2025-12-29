# STM32 Sun Tracking Controller

## Overview
This project contains firmware for an STM32-based sun-tracking controller.
The system reads four photoresistors via the ADC, computes north–south and
east–west light differences, and controls two motors using PWM and H-bridge
direction signals.

## Hardware
- STM32 microcontroller
- Four photoresistors (N/S/E/W)
- Dual H-bridge motor driver
- Linear actuator (N/S axis)
- Worm gear motor (E/W axis)

## Firmware Details
- Multi-channel ADC sampling
- PWM motor control using TIM2
- Direction control via GPIO
- Deadband and saturation handling in control logic
- UART output for debugging and telemetry

## Status
Subsystem-level validation completed.
Sensor input, ADC conversion, PWM generation, and motor direction logic were
verified through bench testing. Full mechanical integration was not completed.
