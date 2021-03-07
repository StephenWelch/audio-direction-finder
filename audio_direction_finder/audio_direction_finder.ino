#include <ArducamSSD1306.h>    // Modification of Adafruit_SSD1306 for ESP8266 compatibility
#include <Adafruit_GFX.h>   // Needs a little change in original Adafruit library (See README.txt file)
#include <Wire.h>           // For I2C comm, but needed for not getting compile error

#include <math.h>

const int OLED_RESET_PIN = 16;
const int OLED_ADDR = 0x78;
const int LEFT_MIC = 2;
const int RIGHT_MIC = 3;
const unsigned long SPEED_OF_SOUND = 343 / pow(10, 9); // Speed of sound in microseconds
const double MIC_DIST = 1.0; // Distance between microphones in meters

ArducamSSD1306 display(OLED_RESET_PIN);

// Variables shared between ISRs and the main loop go here
// These must be marked volatile so that they aren't optimized out by the compiler
// For measuring phase shift
volatile unsigned long left_time, right_time;
// For profiling execution times
volatile unsigned long curr_ts, last_ts, latest_ts_delta;

// Interpolates a parameter based on its possible range of values and a desired range of values
// This is a float-based analogue to the map() function provided by the Arduino standard library
float map_float(float value, float from_low, float from_high, float to_low, float to_high) {
  return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low;
}

void setup() {
  display.begin();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(20,20);

  display.print("Intializing");
  display.display();
  
  // Disable interrupts while we perform hardware setup
  noInterrupts();
  // Setup interrupts to trigger on rising edge of input signal
  attachInterrupt(digitalPinToInterrupt(LEFT_MIC), leftMicIsr, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_MIC), rightMicIsr, RISING);

  // put your setup code here, to run once:
  Serial.begin(115200);

  // Setup is finished - reenable interrupts
  interrupts();

  display.clearDisplay();
}

void loop() {
  // Empty for now - non-time-sensitive processing will eventually go here
  float phase_shift = abs(left_time - right_time);
  float calculated_angle = calculateAngle(phase_shift);

  display.clearDisplay();
  display.setCursor(20,20);
  display.print("Angle: ");
  display.print(millis() / 500);
  display.drawCircle(display.getCursorX() + 3, display.getCursorY(), 2, 1);
  display.display();
//  Serial.print("Phase shift:");
//  Serial.print(phase_shift);
//  Serial.print("Calculated Angle:");
//  Serial.print(calculated_angle);
}

void leftMicIsr() {
  left_time = micros();
}

void rightMicIsr() {
  right_time = micros();
}

// Method stub - calculates angle based on phase shift
float calculateAngle(float phase_shift) {
  return -atan2(phase_shift * SPEED_OF_SOUND, MIC_DIST);
}
