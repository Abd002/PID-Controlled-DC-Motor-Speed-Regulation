# Minia University - CSE311 Project

## Introduction

This repository contains the implementation of a control loop using the PID control algorithm for a DC motor with an encoder mounted to provide feedback and measure motor speed.

### Control Algorithm

The PID control algorithm is employed in the following control loop:

```
𝒖 = 𝑲𝑷 ∗ 𝒆 + 𝑲𝑰 ∫ 𝒆 + 𝑲𝑫
𝒅𝒆
𝒅𝒕
```

- Integration is approximated to a sum and differentiation to a difference.
- At each sample time:
  - Measure the motor speed.
  - Calculate the error by subtracting the reference speed.
  - Calculate the control action 𝒖.

### Implementation Steps

To get started:

1. Measure the motor speed in RPM.
2. Configure timer 0 to operate in FAST PWM for PB3 pin output.
3. The encoder has 360 counts per revolution.
4. Count the pulses within a specified time, divide by 360, then multiply by 60 to get RPM.

## Instructions
- Start by setting KP to 1 and KI, KD to zero.
- Gradually increase KP, KI, KD to fine-tune control action.
- Recommended values for KP, KI, KD: 1.5, 1, 0.5.

## Conclusion

This project aims to implement a PID control algorithm for a DC motor with encoder feedback. Fine-tuning the control constants will ensure optimal performance with minimal overshoot and steady-state error.
