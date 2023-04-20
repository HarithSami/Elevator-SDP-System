# SDP Miniature Elevator Project
This project is a miniature elevator control system developed using an SDP microcontroller. It provides basic functionalities such as stepper and DC motor control, relay testing, PWM control, and more.
![Picture2](https://user-images.githubusercontent.com/66371106/233225188-f72896d8-9452-41dd-8a47-9e564c1f19c3.jpg)

# Features
Stepper motor and DC motor control
Relay testing
Analog to Digital Converter (ADC)
Digital to Analog Converter (DAC)
Pulse Width Modulation (PWM)
Serial Peripheral Interface (SPI)
Timer functions
GPIO testing
Absolute encoder testing

# Files
main.c: Main program file containing the implementation of the project.
Init.h: Header file including all port definitions.
Comms.h: Header file, including comms functions.
ADC.h: Header file including ADC functions.
DAC.h: Header file including DAC functions.
PWM.h: Header file including PWM functions.
SPI.h: Header file including SPI and DRV8711 functions.
Timer.h: Header file including timer functions.
# Usage
The main functionality is provided through a series of tests and options that can be accessed through the console. Users-For more information, see the comments at the beginning of the main.c file.

# Tests and Options
Test relays
Display speed control input value
Display analogue channel 1 input value
Display analogue channel 2 input value
Test DAC output
Test PWM
Test stepper motor
Test DC motor
Read DRV8711 STATUS register
Clear DRV8711 STATUS register
Test GPIO inputs
Test absolute encoder
# Motor Control
The system supports both stepper motor and DC motor control with the ability to set direction, speed, and various other settings. The project includes options to test and display the motor's status.

# General Functions
The project provides support for a variety of general functions such as ADC, DAC, PWM, and SPI. It includes options to test and display the status of these functions.
# Final Project (physical Build) and CAD design
![Picture1](https://user-images.githubusercontent.com/66371106/233225184-eae96898-68bf-4ea2-844c-2541f38f0af8.jpg)


