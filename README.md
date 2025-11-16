# 🐾 Smart Cat Door
## Automated Cat Flap with PIR Sensors & Linear Actuator
This project is a fully automated cat door system powered by an Arduino Uno, two PIR motion sensors (inside + outside), and a 300 mm linear actuator driven by an L298N motor driver.
Totally overengineerd, but cool.

The goal: a silent, smooth, and safe opening/closing mechanism that reacts to your cat’s movements.
The system softly accelerates the actuator to avoid sudden mechanical stress and includes multiple safety layers to prevent injuries or jams.

⚠️ Important: The current setup includes a middle-position switch (swMiddle).
For higher reliability and safety, this switch ***must*** be replaced by an IR break-beam sensor!
This is noted as a required hardware upgrade.

# 🚀 Features
## 🎯 Motion-Triggered Access
- PIR Inside detects when the cat wants to exit.
- PIR Outside detects when the cat wants to enter.
- Automatic direction detection based on which PIR fires.
## 📏 Smooth Motor Movement
- Linear actuator movement is ramped up gradually for quieter and smoother action.
- Prevents mechanical shaking and reduces stress on the system.
## 🛡️ Built-in Safety Functions
- Multiple switches to prevent overtravel.
- Automatic slowdown near end position.
- Automatic stop when the flap is obstructed.
- Planned upgrade to an IR break-beam sensor for more accurate mid-position detection.
## 🐱 Cat-Friendly Behavior
- Flap stays open only as long as needed.
- Closes automatically after a safe timeout.
- Designed to work day/night, even if your cat decides to change its mind mid-motion.

# 🧩 Hardware Overview
## Required Components
- Arduino Uno
- L298N motor driver
- 300 mm linear actuator (12V) > https://amzn.to/3XyPwL6
- 2 × PIR sensors (inside & outside)
- Limit switches/buttons
  - swOpen – detects fully open
  - swClose – detects fully closed
  - swSlowOpenPin - slows the door on opening near the end position
  - swSlowClosePin - slows the door on closing near the end position
  - swMiddle – temporary solution; must be replaced by IR break-beam sensor
  - External 12V DC power supply
  - Breadboard, wiring, resistors, mounting hardware

# Wiring Diagram
A full wiring diagram is included in the repo:
<img src="https://github.com/pocra/katzenklappe/blob/main/Katzenklappe_v3_Steckplatine.png?raw=true">
