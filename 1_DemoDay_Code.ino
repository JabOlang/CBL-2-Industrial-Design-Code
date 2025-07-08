#include <Wire.h>                   // For I2C communication (MPU6050)
#include <Adafruit_NeoPixel.h>      // For controlling WS2812 LEDs
#include <Servo.h>                  // For controlling a servo motor

// ------------------- Pin & Hardware Setup -------------------
#define LED_PIN 5                   // Pin where the LED strip is connected
#define NUMPIXELS 18                // Total number of LEDs in the strip
#define SERVO_PIN 3                 // Pin where the servo motor is connected
#define MPU_ADDR 0x68               // I2C address of the MPU6050 sensor

Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);  // LED object
Servo myServo;                     // Servo motor object

// ------------------- Sensor Data -------------------
int16_t ax, ay, az;                // Raw accelerometer data (x, y, z)
long prevMagnitude = 0;           // Used to detect shaking (motion magnitude)

// ------------------- State & Timing -------------------
float smoothedAngle = 0;          // Smoothed angle to avoid flickering
bool isLogging = false;           // Is an emotion currently being logged?
bool inIdleGlow = false;          // Has the system entered idle ambient state?
unsigned long zoneEnterTime = 0;  // Timestamp when a zone was entered
unsigned long idleStart = 0;      // Timestamp when inactivity started
int lastZone = -1;                // Last detected emotion zone

// ------------------- Emotion Log Data -------------------
int logCount = 0;                 // Number of logged emotions
int cumulativeR = 0, cumulativeG = 0, cumulativeB = 0;  // Color mix of logs

// ------------------- Constants -------------------
const float LOG_THRESHOLD = 1.0;          // Not used currently
const unsigned long HOLD_DURATION = 1000; // Time to hold a tilt to register log
const long IDLE_THRESHOLD = 1000000;      // Threshold to detect inactivity
const unsigned long IDLE_DURATION = 3000; // Idle time to trigger ambient glow

// ------------------- Servo Sweep State -------------------
int servoPos = 150;               // Current position of the servo
int servoDir = 1;                 // Direction of servo movement (+1 or -1)
unsigned long lastServoMove = 0;  // Timestamp of last servo move
const int sweepMin = 30;          // Minimum servo angle
const int sweepMax = 70;          // Maximum servo angle
const int servoStep = 3;          // Servo angle step size
const int servoDelay = 15;        // Delay between servo steps (ms)

// ------------------- Color Zones (Tilt Directions) -------------------
enum Zone { RED = 0, GREEN, BLUE, YELLOW };
uint32_t zoneColors[4] = {
  pixels.Color(255, 0, 0),     // RED
  pixels.Color(0, 255, 0),     // GREEN
  pixels.Color(0, 0, 255),     // BLUE
  pixels.Color(255, 255, 0)    // YELLOW
};

// ------------------- Setup -------------------
void setup() {
  Serial.begin(9600);
  Wire.begin(); // Start I2C communication

  // Wake up the MPU6050 (exit sleep mode)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); 
  Wire.write(0);     
  Wire.endTransmission(true);

  pixels.begin();     // Initialize LED strip
  pixels.show();      // Clear all LEDs

  myServo.attach(SERVO_PIN); // Attach the servo to its pin
  myServo.write(servoPos);   // Move servo to initial position
}

// ------------------- Main Loop -------------------
void loop() {
  readMPU();          // Get accelerometer data
  sweepServo();       // Continuously move the servo back and forth

  // Measure change in motion to detect shake
  long currentMagnitude = (long)ax * ax + (long)ay * ay + (long)az * az;
  long delta = abs(currentMagnitude - prevMagnitude);
  prevMagnitude = currentMagnitude;

  // --- Shake Reset ---
  if (delta > 600000000) {
    resetState(); // Shake detected → reset emotion log
    return;
  }

  // --- Idle Detection ---
  if (delta < IDLE_THRESHOLD && logCount > 0) {
    if (idleStart == 0) idleStart = millis(); // Start timer
    else if (!inIdleGlow && millis() - idleStart >= IDLE_DURATION) {
      inIdleGlow = true;
      fadeToStaticCumulative(); // Transition to calm glow
    }
  } else {
    idleStart = 0;
    inIdleGlow = false;
  }

  // --- Ambient Glow (after 8 logs) ---
  if (inIdleGlow && logCount >= 8) {
    playAmbientGlow(); // Play looping light animation
    return;
  }

  // --- Main LED Tilt Animation ---
  showTiltEffect();
  delay(50); // Slight pause to make transitions smoother
}

// ------------------- Function to Read MPU6050 Sensor -------------------
void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Starting register for accelerometer
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true); // Read 14 bytes

  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // Skip temperature
  Wire.read(); Wire.read(); // Skip gyro X
  Wire.read(); Wire.read(); // Skip gyro Y
  Wire.read(); Wire.read(); // Skip gyro Z
}

// ------------------- LED Visualization Based on Tilt -------------------
void showTiltEffect() {
  float angle = atan2((float)az, (float)ax); // Compute tilt angle
  if (angle < 0) angle += 2 * PI;

  // Smooth angle to prevent jitter
  float alpha = 0.1;
  smoothedAngle = (1 - alpha) * smoothedAngle + alpha * angle;

  int zone = ((int)(smoothedAngle / (2 * PI) * 4)) % 4; // Determine zone
  float pos = zone * (NUMPIXELS / 4.0); // Position on LED strip

  // Animate LED trail around the tilt direction
  for (int i = 0; i < NUMPIXELS; i++) {
    float dist = min(abs(i - pos), NUMPIXELS - abs(i - pos));
    float brightness = exp(-0.3 * dist); // Fade based on distance
    uint32_t color = zoneColors[zone];
    uint8_t r = ((color >> 16) & 0xFF) * brightness;
    uint8_t g = ((color >> 8) & 0xFF) * brightness;
    uint8_t b = (color & 0xFF) * brightness;
    pixels.setPixelColor(i, pixels.Color(r, g, b));
  }
  pixels.show();

  // Emotion Logging
  if (zone != lastZone) {
    lastZone = zone;
    zoneEnterTime = millis();
    isLogging = false;
  } else if (!inIdleGlow && !isLogging && millis() - zoneEnterTime >= HOLD_DURATION) {
    isLogging = true;
    registerEmotion(zone);              // Add emotion color
    softPulseAll(zoneColors[zone]);     // Feedback pulse

    if (logCount >= 8) {
      fadeToStaticCumulative();
      playAmbientGlow();
      resetState();
    }
  }
}

// ------------------- Emotion Logging & Color Mixing -------------------
void registerEmotion(int zone) {
  logCount++;
  switch (zone) {
    case RED:    cumulativeR += 32; break;
    case GREEN:  cumulativeG += 32; break;
    case BLUE:   cumulativeB += 32; break;
    case YELLOW: cumulativeR += 16; cumulativeG += 16; cumulativeB += 16; break;
  }

  // Cap values at max brightness
  cumulativeR = min(cumulativeR, 255);
  cumulativeG = min(cumulativeG, 255);
  cumulativeB = min(cumulativeB, 255);
}

// ------------------- Fade into Static Cumulative Glow -------------------
void fadeToStaticCumulative() {
  for (int step = 0; step <= 255; step += 5) {
    uint8_t r = (cumulativeR * step) / 255;
    uint8_t g = (cumulativeG * step) / 255;
    uint8_t b = (cumulativeB * step) / 255;
    for (int i = 0; i < NUMPIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(r, g, b));
    }
    pixels.show();
    delay(30);
  }
}

// ------------------- Ambient Breathing Glow -------------------
void playAmbientGlow() {
  static float offset = 0;
  offset += 0.15;
  if (offset >= NUMPIXELS) offset -= NUMPIXELS;

  for (int i = 0; i < NUMPIXELS; i++) {
    float phase = sin((i + offset) * 0.4) * 0.5 + 0.5;
    uint8_t r = (uint8_t)(cumulativeR * phase);
    uint8_t g = (uint8_t)(cumulativeG * phase);
    uint8_t b = (uint8_t)(cumulativeB * phase);
    pixels.setPixelColor(i, pixels.Color(r, g, b));
  }
  pixels.show();
  delay(60);
}

// ------------------- Servo Sweep Animation -------------------
void sweepServo() {
  if (millis() - lastServoMove >= servoDelay) {
    servoPos += servoDir * servoStep;
    if (servoPos >= sweepMax || servoPos <= sweepMin) {
      servoDir *= -1;
      servoPos = constrain(servoPos, sweepMin, sweepMax);
    }
    myServo.write(servoPos);
    lastServoMove = millis();
  }
}

// ------------------- Soft Pulse Animation for Logging Feedback -------------------
void softPulseAll(uint32_t color) {
  for (int p = 0; p < 2; p++) {
    for (int b = 0; b <= 255; b += 25) {
      setAllColorWithBrightness(color, b);
      delay(15);
    }
    for (int b = 255; b >= 0; b -= 25) {
      setAllColorWithBrightness(color, b);
      delay(15);
    }
  }
  delay(100);
}

void setAllColorWithBrightness(uint32_t color, uint8_t brightness) {
  uint8_t r = ((color >> 16) & 0xFF) * brightness / 255;
  uint8_t g = ((color >> 8) & 0xFF) * brightness / 255;
  uint8_t b = (color & 0xFF) * brightness / 255;
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(r, g, b));
  }
  pixels.show();
}

// ------------------- Reset Everything on Shake -------------------
void resetState() {
  logCount = 0;
  cumulativeR = cumulativeG = cumulativeB = 0;
  isLogging = false;
  inIdleGlow = false;
  idleStart = 0;
  runQuickRainbow(); // Play rainbow animation on reset
}

// ------------------- Rainbow Animation for Reset Feedback -------------------
void runQuickRainbow() {
  for (int cycle = 0; cycle < 256; cycle += 20) {
    for (int i = 0; i < NUMPIXELS; i++) {
      pixels.setPixelColor(i, Wheel((i * 256 / NUMPIXELS + cycle) & 255));
    }
    pixels.show();
    delay(100);
  }
}

// ------------------- Color Wheel Helper Function -------------------
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) return pixels.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  if (WheelPos < 170) {
    WheelPos -= 85;
    return pixels.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return pixels.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
