# Custom Model Train System With Pan-Tilt Tracking, Positioning, and Aiming Sentries

This repository contains the codebase for a custom model train system control. The system uses two pan-tilt turrets, one for searching and tracking a train as it follows a track, and the other for hitting the train with a laser from any region of the track.

## System Overview

The system is designed around two custom 3D printed pan-tilt turrets, each equipped with a NEMA 17 stepper motor, variable current motor drivers, and AS5600 motor encoders. The turrets use specific ratio gear pulleys for precise control.

The train system is controlled by a Hornbee train controller system and a custom microprocessor implemented on a scotchflex vector board consisting of an EEPROM, ACIA, and other components. The system interfaces with two ESP32-S2-Minis and custom power sensor fusion and motor control boards.

## Features

- **Train Tracking**: Uses a PixyCam and a Time-of-Flight (ToF) sensor to search and track the train. The system outputs the global coordinates of the train via ESP32.
- **Laser Control**: Takes in the reported coordinates of the train and controls a laser to hit the train from any region of the track.

## Hardware Requirements

- ESP32-S2-Minis
- PixyCam
- Time-of-Flight (ToF) sensor
- Pan-Tilt Turrets (Custom 3D printed)
- NEMA 17 Stepper Motors
- Variable Current Motor Drivers
- AS5600 Motor Encoders
- Specific Ratio Gear Pulleys
- Laser Pointer
- Hornbee Train Controller System
- Custom Microprocessor (Implemented on a Scotchflex Vector Board)
- Custom Power Sensor Fusion and Motor Control Boards

## Software Requirements

- Arduino IDE
- VL53L0X Library
- Pixy Library
- BasicStepperDriver Library

## Setup

1. Connect the PixyCam and ToF sensor to the ESP32.
2. Connect the pan-tilt turrets to the ESP32.
3. Connect the laser pointer to the ESP32.
4. Open the `mae412_group5.ino` file in the Arduino IDE.
5. Upload the code to the ESP32.

## Usage

The system operates in three states:

- **Inactive**: The system is idle and the steppers are disabled.
- **Search**: The system searches for the train using the PixyCam and ToF sensor. The steppers are enabled and move the turret to track the train.
- **Lock**: The system locks onto the train and the laser pointer is activated. The turret moves to follow the train and the laser pointer adjusts to hit the train.

## Contributing

Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.
