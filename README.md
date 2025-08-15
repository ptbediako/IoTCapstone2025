# IoTCapstone2025
### Authored by Phylicia Bediako
### Hackster: 
https://www.hackster.io/ptbediako/iot-smart-grocery-bag-4920f7

## Smart Grocery Bag: Intro and Motivation
Have you ever wondered if your groceries were still safe enough to eat after a long day of running errands, especially in the heat? Have you ever lost your cherished snacks or treats to a bag or container that tipped over or leaked silently while out of sight? The Smart Grocery Bag was created to help you know what's happening to your food while in transit so you can enjoy the fruits of your shopping and food transportation labor. 

## Main Components
* Particle Photon2
* BME 280
* OLED
* MPU 6050 Accelerometer
* Latching Button
* Water Sensor
* Li-Ion Battery
* Solder board

## Equipment
* Bambu Lab X1 Carbon 3D Printer
* Dremel
* Rotary saw

## Software
* C/C++ in Visual Studio Code
* SolidWorks
* Bambu Studio
* Adobe Premier Rush
* Monday.com
* Lucidchart

## Features
* Real time simultaneous readings of the temperatures inside the bag and the surrounding environment
* Bag falling or leaning detector
* Detects spill or leak in bag and displays messages on OLED and dashboard
* Detects when the bag has been shaken too much and initiates a warning/countdown timer until things have settled
* OLED and Adafruit dashboard options for keeping track of bag conditions
* Manual power button

## How it works and what it does
The smart bag uses two BME 280s on opposite sides of a solder board to measure temperature inside the bag and in the immediate environment around the bag. An accelerometer measures whether the bag is standing up, leaning, or falling over. A water sensor is attached to the bottom of the bag to detect leaks and spills. The device can connect to a mobile hotspot for on-the-go monitoring of conditions. When connected to internet, the device publishes to an Adafruit dashboard that can be accessed via phone of computer. The device includes a manual power button attached to the front of the control panel.
