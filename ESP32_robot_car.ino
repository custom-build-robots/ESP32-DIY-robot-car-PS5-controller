/***************************************************
 * Author: Ingmar Stapel
 * Website: www.custom-build-robots.com
 * Date: 2024-12-15
 * Version: 1.2
 * Description:
 * This program enables your ESP32-based robot car to be
 * controlled via a PlayStation 4 or 5 controller using the
 * Bluepad32 library. Includes an inversion option
 * for adaptable control directions and a deadzone for
 * joystick neutrality.Now includes WS2812 LED ring
 * control with multiple lighting modes.
 ****************************************************/

#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "SSD1306Wire.h" // For OLED display
#include <Bluepad32.h>   // Include Bluepad32 library
#include <Adafruit_NeoPixel.h> // For WS2812 LEDs

// ----------------------------
// Initialize the OLED display using Wire library
// ----------------------------
SSD1306Wire display(0x3c, 21, 22); // SDA=21, SCL=22 for ESP32

// ----------------------------
// PCA9685 Servo Controller
// ----------------------------
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ----------------------------
// Robot Name
// ----------------------------
const String ROBOT_NAME = "Blitzy rush";

// ----------------------------
// Motor Speeds (-100 to 100)
// ----------------------------
int speed_left = 0;
int speed_right = 0;

// ----------------------------
// Command Timeout for Safety
// ----------------------------
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT = 5000; // 5 seconds

// ----------------------------
// PWM Constants
// ----------------------------
const int MAX_PWM = 4095;

// ----------------------------
// Inversion Option
// ----------------------------
const bool invert_controls = false; // Set to 'true' to invert controls

// ----------------------------
// Direction Multiplier
// ----------------------------
const int direction_multiplier = invert_controls ? -1 : 1;



// ----------------------------
// NeoPixel LED Ring Setup
// ----------------------------
#define LED_PIN 16
#define NUM_LEDS 24
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

enum LedMode {
  OFF_MODE,
  AMBIENT_MODE,
  SPEED_INDICATOR_MODE,
  POLICE_MODE
};

LedMode currentLedMode = AMBIENT_MODE; // Start with ambient mode

// For button state tracking
bool prev_a = false;
bool prev_b = false;
bool prev_x = false;
bool prev_y = false;


// ----------------------------
// Deadzone Configuration
// ----------------------------
const int DEADZONE = 10; // Deadzone range (-10 to 10)

// ----------------------------
// Bluepad32 Controller Setup
// ----------------------------
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("Controller connected, index=%d\n", i);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("Controller connected, but no empty slot found");
  }

  // Display controller connection status on OLED
  display.clear();
  display.drawString(0, 0, ROBOT_NAME);
  display.drawString(0, 20, "Controller Connected");
  display.display();
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("Controller disconnected, index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }
  if (!foundController) {
    Serial.println("Controller disconnected, but not found in myControllers");
  }

  // Stop the motors when controller disconnects
  speed_left = 0;
  speed_right = 0;
  updateMotorPWM("left", speed_left);
  updateMotorPWM("right", speed_right);
  Serial.println("Controller disconnected. Motors stopped.");

  // Update OLED to reflect controller disconnection
  display.clear();
  display.drawString(0, 0, ROBOT_NAME);
  display.drawString(0, 20, "Controller Disconnected");
  display.display();
}

void processControllers() {
  for (auto ctl : myControllers) {
    if (ctl && ctl->isConnected() && ctl->hasData()) {
      if (ctl->isGamepad()) {
        processGamepad(ctl);
      }
    }
  }
}

void processGamepad(ControllerPtr ctl) {
  int16_t axisX = ctl->axisX(); // Left stick horizontal (-511 to 512)
  int16_t axisY = ctl->axisY(); // Left stick vertical (-511 to 512)

  // Normalize axis values to -100 to 100
  int speed = -axisY * 100 / 512; // axisY negative when stick up
  int turn = axisX * 100 / 512;   // axisX negative when stick left

  // Apply deadzone
  if (abs(speed) < DEADZONE) {
    speed = 0;
  }
  if (abs(turn) < DEADZONE) {
    turn = 0;
  }

  // Apply direction multiplier for inversion
  speed *= direction_multiplier;
  turn *= direction_multiplier;

  // Calculate left and right motor speeds
  speed_left = speed + turn;
  speed_right = speed - turn;

  // Constrain speeds to -100 to 100
  speed_left = constrain(speed_left, -100, 100);
  speed_right = constrain(speed_right, -100, 100);

  // Update motor speeds
  updateMotorPWM("left", speed_left);
  updateMotorPWM("right", speed_right);

  lastCommandTime = millis(); // Reset command timeout

  // Optional: Print speeds for debugging
  Serial.printf("Speed Left: %d, Speed Right: %d\n", speed_left, speed_right);


  // Handle LED mode switching
  if (ctl->x() && !prev_x) {
    currentLedMode = AMBIENT_MODE;
    Serial.println("LED Mode: AMBIENT_MODE");
  }

  if (ctl->y() && !prev_y) {
    currentLedMode = SPEED_INDICATOR_MODE;
    Serial.println("LED Mode: SPEED_INDICATOR_MODE");
  }

  if (ctl->b() && !prev_b) {
    currentLedMode = POLICE_MODE;
    Serial.println("LED Mode: POLICE_MODE");
  }

  if (ctl->a() && !prev_a) {
    currentLedMode = OFF_MODE;
    Serial.println("LED Mode: OFF_MODE");
  }

  // Update previous button states
  prev_x = ctl->x();
  prev_y = ctl->y();
  prev_b = ctl->b();
  prev_a = ctl->a();

}

// ----------------------------
// Setup Function
// ----------------------------
void setup() {
  // Initialize Serial Communication
  Serial.begin(115200);
  Serial.println();
  Serial.println("Initializing Robot Car Control...");

  Serial.println("Initializing OLED Display");
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);

  // Display Robot Name
  display.drawString(0, 0, ROBOT_NAME);
  display.drawString(0, 20, "Initializing...");
  display.display();

  // Initialize PCA9685
  pwm.begin();
  pwm.setOscillatorFrequency(25000000);
  pwm.setPWMFreq(100); // 100 Hz for motor control
  Wire.setClock(400000);

  // Initialize Bluepad32
  Serial.println("Initializing Bluepad32...");
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // Optionally, forget Bluetooth keys if needed
  // BP32.forgetBluetoothKeys();

  // Disable virtual devices (e.g., mouse, keyboard)
  BP32.enableVirtualDevice(false);

  // Display initialization complete
  display.clear();
  display.drawString(0, 0, ROBOT_NAME);
  display.drawString(0, 20, "Waiting for Controller");
  display.display();

  Serial.println("Setup complete. Waiting for controller...");
}

// ----------------------------
// Loop Function
// ----------------------------
void loop() {
  // Update Bluepad32
  bool dataUpdated = BP32.update();
  if (dataUpdated) {
    processControllers();
  }

  // Safety: Stop motors if no commands received recently
  if (millis() - lastCommandTime > COMMAND_TIMEOUT) {
    if (speed_left != 0 || speed_right != 0) {
      speed_left = 0;
      speed_right = 0;
      updateMotorPWM("left", speed_left);
      updateMotorPWM("right", speed_right);
      Serial.println("Command timeout. Motors stopped.");

      // Update OLED to reflect motors stopped
      display.clear();
      display.drawString(0, 0, ROBOT_NAME);
      display.drawString(0, 20, "Motors Stopped");
      display.display();
    }
  }

  updateLEDs(); // Update the LEDs based on current mode

  // Yield to lower priority tasks
  delay(1);
}


// ----------------------------
// Update LEDs Based on Current Mode
// ----------------------------
void updateLEDs() {
  static uint32_t previousMillis = 0;
  uint32_t currentMillis = millis();

  switch (currentLedMode) {
    case OFF_MODE:
      // Turn off all LEDs
      strip.clear();
      strip.show();
      break;

    case AMBIENT_MODE:
      // Implement color cycling or rainbow effect
      if (currentMillis - previousMillis >= 50) { // Update every 50ms
        previousMillis = currentMillis;
        static uint16_t hue = 0;
        for (int i = 0; i < strip.numPixels(); i++) {
          // Create a rainbow effect along the strip
          uint32_t color = strip.gamma32(strip.ColorHSV(hue + (i * 65536L / strip.numPixels())));
          strip.setPixelColor(i, color);
        }
        strip.show();
        hue += 256; // Increment hue
      }
      break;

    case SPEED_INDICATOR_MODE:
      // Indicate speed and direction
      // Update only when speed changes
      static int prev_speed_left = 0;
      static int prev_speed_right = 0;
      if (speed_left != prev_speed_left || speed_right != prev_speed_right) {
        prev_speed_left = speed_left;
        prev_speed_right = speed_right;

        // Calculate average speed
        int avg_speed = (abs(speed_left) + abs(speed_right)) / 2;

        // Map speed to number of LEDs to light up
        int num_leds = map(avg_speed, 0, 100, 0, strip.numPixels());

        // Set color based on direction
        uint32_t color;
        if (speed_left > 0 && speed_right > 0) {
          // Forward - Green
          color = strip.Color(0, 255, 0);
        } else if (speed_left < 0 && speed_right < 0) {
          // Backward - Red
          color = strip.Color(255, 0, 0);
        } else if (speed_left > speed_right) {
          // Turning Right - Blue
          color = strip.Color(0, 0, 255);
        } else if (speed_left < speed_right) {
          // Turning Left - Yellow
          color = strip.Color(255, 255, 0);
        } else {
          // Stopped or other - White
          color = strip.Color(255, 255, 255);
        }

        // Light up LEDs
        strip.clear();
        for (int i = 0; i < num_leds; i++) {
          strip.setPixelColor(i, color);
        }
        strip.show();
      }
      break;

    case POLICE_MODE:
      // Simulate police lights
      if (currentMillis - previousMillis >= 200) { // Update every 200ms
        previousMillis = currentMillis;
        static bool toggle = false;
        strip.clear();
        if (toggle) {
          // Blue flash
          for (int i = 0; i < strip.numPixels() / 2; i++) {
            strip.setPixelColor(i, strip.Color(0, 0, 255));
          }
        } else {
          // Red flash
          for (int i = strip.numPixels() / 2; i < strip.numPixels(); i++) {
            strip.setPixelColor(i, strip.Color(255, 0, 0));
          }
        }
        strip.show();
        toggle = !toggle;
      }
      break;
  }
}

// ----------------------------
// Set Motor Direction
// ----------------------------
void set_motor_mode(String motor, String motor_direction) {
  if (motor == "left") {
    if (motor_direction == "forward") {
      pwm.setPWM(1, 0, 4096);
      pwm.setPWM(2, 4096, 0);
      Serial.println("left | forward");
    } else if (motor_direction == "backward") {
      pwm.setPWM(1, 4096, 0);
      pwm.setPWM(2, 0, 4096);
      Serial.println("left | backward");
    }
  }

  if (motor == "right") {
    if (motor_direction == "forward") {
      pwm.setPWM(4, 0, 4096);
      pwm.setPWM(3, 4096, 0);
      Serial.println("right | forward");
    } else if (motor_direction == "backward") {
      pwm.setPWM(4, 4096, 0);
      pwm.setPWM(3, 0, 4096);
      Serial.println("right | backward");
    }
  }
}

// ----------------------------
// Update Motor PWM Based on Speed
// ----------------------------
void updateMotorPWM(String motor, int speed) {
  if (motor == "right") {
    if (speed > 0) {
      set_motor_mode("left", "forward");
      pwm.setPWM(0, 0, map(speed, 0, 100, 0, MAX_PWM));
      Serial.println("Left PWM: " + String(map(speed, 0, 100, 0, MAX_PWM)));
    } else if (speed < 0) {
      set_motor_mode("left", "backward");
      pwm.setPWM(0, 0, map(-speed, 0, 100, 0, MAX_PWM));
      Serial.println("Left PWM: " + String(map(-speed, 0, 100, 0, MAX_PWM)));
    } else {
      // Stop the motor
      set_motor_mode("left", "backward"); // Assuming stopping by setting both directions to backward
      pwm.setPWM(0, 0, 0); // Stop PWM signal
      Serial.println("Left Motor Stopped.");
    }
  } else if (motor == "left") {
    if (speed > 0) {
      set_motor_mode("right", "forward");
      pwm.setPWM(5, 0, map(speed, 0, 100, 0, MAX_PWM));
      Serial.println("Right PWM: " + String(map(speed, 0, 100, 0, MAX_PWM)));
    } else if (speed < 0) {
      set_motor_mode("right", "backward");
      pwm.setPWM(5, 0, map(-speed, 0, 100, 0, MAX_PWM));
      Serial.println("Right PWM: " + String(map(-speed, 0, 100, 0, MAX_PWM)));
    } else {
      // Stop the motor
      set_motor_mode("right", "backward"); // Assuming stopping by setting both directions to backward
      pwm.setPWM(5, 0, 0); // Stop PWM signal
      Serial.println("Right Motor Stopped.");
    }
  }
}
