# MINI-ECU: Bare-Metal Embedded Drive Control System

<img width="2560" height="1444" alt="image" src="https://github.com/user-attachments/assets/424b2ba4-43d1-42ae-8e22-7dc187d990fc" />


*Illustrative project overview; actual prototype hardware may differ.*

## Problem Statement

The Silicon Sprint Embedded Systems challenge is to design a bare-metal Embedded Drive Control Unit (MINI-ECU) using an STM32 microcontroller to control a small, two-motor vehicle through user inputs and PC-based UART commands.

The system must provide controlled vehicle movement, variable speed selection, clear operating-state indication, and a way to stop the motors during normal and emergency conditions. The implementation is developed in bare-metal C, without an RTOS, Arduino framework, or high-level motor-control libraries.

The required behaviour includes:
- Keep the motors disabled after power-up until the user enables the system.
- Read a potentiometer through the ADC to set the requested motor speed.
- Accept movement and stop commands through UART.
- Use a finite state machine (FSM) to manage IDLE, READY, RUNNING, and emergency-stop states.
- Immediately disable motor drive when the physical E-STOP is triggered.
- Provide LED indications and UART status reporting so the current system state can be monitored.

## Working Logic

The MINI-ECU runs as a bare-metal application. After power-up, it initializes the required peripherals—including GPIO, ADC, UART, PWM timer, SysTick, and the external-interrupt input—and places the motor outputs in the OFF state. The controller then manages the vehicle using an FSM and continuously checks user input, UART commands, speed selection, and safety conditions.

### 1. System enable and state management

The application uses four states:
- **IDLE:** The controller is initialized, motors are OFF, and the system waits for the user to enable it.
- **READY:** The system is enabled and waits for a valid movement command.
- **RUNNING:** The motor driver is active and the vehicle moves in the selected direction.
- **ESTOP:** An emergency condition is active and motor drive is disabled.

On startup, the system enters IDLE. Touching the START sensor enables the system and moves it to READY. A valid UART movement command can move the controller from READY to RUNNING when the requested PWM is non-zero and no emergency is active. A stop command, or a zero PWM request while running, stops the motors and returns the controller to READY.

### 2. Speed selection and motor control

A potentiometer provides the speed request. Its analog voltage is read by the ADC, filtered, and mapped to a PWM duty command between 0% and 100%. The potentiometer sets the requested speed; it does not start the motors by itself.

When a valid movement command is received, the controller applies the requested direction and PWM through the motor-driver module. The PWM signal controls the motor drive level, while the direction signals select forward or reverse operation. A configured dead time is applied during a direction change before the new direction is driven.

### 3. UART command handling

The controller receives commands from a PC terminal over UART. The supplied command interface supports:
- **`F`** — request forward movement.
- **`R`** — request reverse movement, subject to the motor configuration.
- **`S`** — stop the motors and return to READY.
- **`STATUS`** — report the current state and operating values.
- **`RESET`** — clear an emergency-stop condition only after the physical E-STOP has been released; the system returns to IDLE and requires a fresh START action.

Movement commands are not accepted if the system is not enabled, the emergency stop is active, or the selected PWM is zero.

### 4. Emergency-stop logic

The physical E-STOP input is handled through an external interrupt. When triggered, the interrupt immediately disables the motor outputs, clears the stored PWM and direction request, sets the emergency condition, and places the FSM in ESTOP. The Red LED indicates the emergency state.

After the physical E-STOP button has been released, the user can send `RESET` over UART. This clears the emergency condition and returns the controller to IDLE—not directly to RUNNING. The user must touch START again before issuing a movement command.

### 5. Status indication and monitoring

LEDs indicate the controller's current operating state: Orange for IDLE, Green for READY, Blue for RUNNING, and Red for ESTOP. The `STATUS` UART command reports values such as the FSM state, system enable status, motor status, direction, PWM request, ADC reading, and emergency status.

This separation between input handling, state/safety logic, and low-level motor control keeps normal movement commands subject to the same enable and emergency checks.

## YouTube Playlist

[Watch the MINI-ECU project playlist](https://www.youtube.com/playlist?list=PLdXBl2AQIudM)
