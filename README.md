======================================================================
README — EMOTION LOGGING COMPANION: TILT + LED + SERVO + AMBIENT GLOW
======================================================================

This Arduino program is designed for a our physical-digital companion that helps
users log emotions by tilting a cup embedded with sensors, an LED strip, and a 
servo motor. The prototype uses an MPU6050 motion sensor to detect tilt 
direction and maps it to one of four "emotion zones" (Red, Green, Blue, Yellow).

Key Behaviors:
---------------
1. **Tilt Detection** — The MPU6050 detects the companion's orientation and assigns
   it to one of four 90° zones.
2. **LED Animation** — The LED ring lights up in a gradient toward the detected
   tilt direction and corresponding emotion color.
3. **Emotion Logging** — Holding the tilt steadily in a zone for 1 second logs
   the corresponding emotion and triggers a soft LED pulse.
4. **Ambient Glow** — After 8 logs, the LEDs display a calming animation using 
   the cumulative mix of logged colors.
5. **Shake Reset** — A shake motion resets all data and plays a rainbow animation.
6. **Servo Movement** — A servo continuously sweeps to add a sense of motion.

This prototype is intended to evoke emotional self-awareness and delight through
motion, light, and interaction. Ideal for design students, emotion-based feedback
systems, or as a research tool.

Hardware Requirements:
-----------------------
- MPU6050 motion sensor (I2C)
- WS2812 (NeoPixel) LED strip (18 LEDs)
- Servo motor
- Arduino Uno

Code Support Acknowledgement
----------------------------
This script was developed with the assistance of ChatGPT by OpenAI.
ChatGPT was used for debugging, condensing complex logic, and improving
code structure for readability and performance.
For transparency and future reference, a QR code linking to the full
source code is included in the research workbook.
