# Speed-Measurer
An embedded system that reads and displays the speed of an object.

## Overview
The Speed-Measurer was inspired by school zone speed displays and serves as a learning project for embedded systems. It builds upon a simpler setup that measured distance using an Arduino Uno, an ultrasonic sensor, and a buzzer. This project measures the speed of objects, displays their speed and distance on a GLCD screen, and alerts users when speed limits are exceeded.

**Key Features:**
- Displays speed and distance on a GLCD screen.
- Allows users to set the speed limit via a 4x4 keypad.
- Alerts using an LED:
  - LED brightness set to 10% if an object is moving over 80% of the speed limit but not speeding.
  - LED brightness set to 100% if an object exceeds the speed limit.

[Watch the Software Demo Video](https://www.youtube.com)

## Development Environment
The project was developed using the following tools and techniques:
- **IDE**: STM32 Cube IDE
- **Programming Language**: C
- **Communication Protocol**: SPI (for GLCD)
- **Interrupts**: Used for keypad input
- **Timers and Pulse Width Modulation**: Implemented via Capture/Compare Register

## Parts List
To replicate or build upon the Speed-Measurer, you’ll need:
- Nucleo-L476RG microcontroller
- HC-SR04 ultrasonic sensor
- PCD8544 GLCD screen
- 4x4 matrix keypad
- 74c922 keypad encoder IC
- 100-ohm resistor
- 1 µF capacitor
- 10 µF capacitor
- Red LED
- ≈35 jumper wires
- Breadboard

## Useful References
Learn more about the components and implementation through these resources:
- [Ultrasonic Sensor with STM32 Nucleo Board](https://microcontrollerslab.com/hc-sr04-ultrasonic-sensor-stm32-nucleo-stm32cubeide/)
- [Ultrasonic Sensor with Arduino](https://ecelabs.njit.edu/fed101/resources/HC-SR04%20Ultrasonic%20Sensor.pdf)

## Future Enhancements
The project has several avenues for improvement:
- **Improved Sensors**: Use sensors with greater distance accuracy.
- **Higher Speed Limits**: Allow for speed limits beyond 9 mph.
- **Enclosed Design**: Reorganize components into a compact, enclosed product.
- **Camera Integration**: Add a camera to capture images of speeding objects.
- **WiFi Module**: Enable connectivity for uploading captured images.
