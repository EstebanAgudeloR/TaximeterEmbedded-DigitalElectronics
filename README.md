Taximeter with STM32 (C)
Overview:
This project implements a taximeter system on an STM32 microcontroller using C.
The system simulates the behavior of a real taximeter by combining:

-State machine architecture for event handling.
-Interrupt-driven programming using hardware timers (TIM2, TIM3).
-Digital display control through multiplexed 7-segment displays.
-Rotary encoder input for real-time user interaction.
-RGB LED feedback controlled by software-defined cases.

The focus of this project is not only on the hardware interaction, but also on the programming logic, modular design, and resource management in embedded systems.

Key Features:
-Finite State Machine (FSM):
-Centralized state handling (IDLE, ENCODER, SWITCH, REFRESH) that ensures modularity and readability.
Interrupt-driven design:
-Uses hardware timers to manage periodic events and display refresh without blocking loops.
Efficient display multiplexing:
-Converts a 12-bit counter (0–4095) into 4-digit BCD and dynamically updates the 7-segment displays.

User interaction:
-Rotary encoder controls the counter.
-Button cycles through LED RGB states.

Code structure:
Functions are clearly separated (displayNumber, refreshDisplay, getDigit, colorRGB), highlighting clean modular programming.

Notes
This project gave me practice in:
Writing modular code in C for embedded systems.
Using timers, interrupts, and GPIOs to synchronize hardware events.
Applying state machine design for control logic.
Managing resources and tasks efficiently in real time.




