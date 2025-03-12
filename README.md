Inclinometer

**Arduino Leveling Device with MPU6050 & OLED**
This project implements a leveling device using an MPU6050 sensor and an SSD1306 OLED display. It features real-time angle measurement with both normal and precision display modes, an interactive menu for mode selection and calibration, and EEPROM-based calibration storage for persistent settings.

**Features**
Real-Time Angle Measurement:
Uses the MPU6050 sensor's Digital Motion Processing (DMP) to obtain quaternion data and convert it to Euler angles for accurate leveling.

**Calibration with EEPROM Storage:**
Enables sensor calibration to account for offsets. Calibration values are saved to EEPROM so they persist between power cycles.

**Interactive Menu System:**
A simple menu navigated using two push buttons lets you choose between Normal mode, Precision mode, and a Calibrate option.

**Multi-Mode Display:**

i)Normal Mode: Shows rounded angle values.

ii)Precision Mode: Displays angle values with one decimal place for finer resolution.
