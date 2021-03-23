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
volatile unsigned long left_time, right_time, last_left_time, last_right_time, phase_shift;
volatile bool recalc = false;
// For profiling execution times
//volatile unsigned long curr_ts, last_ts, latest_ts_delta;

typedef struct coord {
  double x, y;

  coord operator+(const coord &other) {
    return {x + other.x, y + other.y};
  }

  coord operator-(const coord &other) {
    return {x - other.x, y - other.y};
  }

  coord operator-() {
    return {-x, -y};
  }

  coord operator*(float scalar) {
    return {x * scalar, y * scalar};
  }

  coord rotate(double angle) {
    return {x * cos(angle) - y * sin(angle), x * sin(angle) + y * cos(angle)};
  }
  
  coord rotate(const coord &origin, double angle) {
    return (*this - origin).rotate(angle) + origin;
  }
  
} coord_t;

coord_t line_start = {64, 32};
coord_t line_end = {64, 64};
coord_t arrow_head_l_end = line_start + (coord_t){-8, 8};
coord_t arrow_head_r_end = line_start + (coord_t){8, 8};

coord_t origin = {64, 48};

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

//  display.clearDisplay();
}

void loop() {
  // Empty for now - non-time-sensitive processing will eventually go here
//  float calculated_angle = calculateAngle(phase_shift);
  float calculated_angle = millis() / 250;  
  display.clearDisplay();
  display.setCursor(20,20);
  displayAngleOled(calculated_angle);
  display.setCursor(64, 64);

  float calculated_angle_rad = calculated_angle * PI / 180.0;
  displayLineOled(line_start.rotate(origin, calculated_angle_rad), line_end.rotate(origin, calculated_angle_rad), WHITE);
  displayLineOled(line_start.rotate(origin, calculated_angle_rad), arrow_head_l_end.rotate(origin, calculated_angle_rad), WHITE);
  displayLineOled(line_start.rotate(origin, calculated_angle_rad), arrow_head_r_end.rotate(origin, calculated_angle_rad), WHITE);
  display.display();
//  if(recalc) {
//    phase_shift = abs((float)left_time - right_time);
//    Serial.print("Phase shift:");
//    Serial.print(phase_shift);
//    Serial.print(" ");
//    Serial.print("Left Period:");
//    Serial.print(abs((float)left_time - last_left_time));
//    Serial.print(" ");
//    Serial.print("Right Period:");
//    Serial.print(abs((float)right_time - last_right_time));
//  //  Serial.print("Calculated Angle:");
//  //  Serial.print(calculated_angle);
//    Serial.println();
//    recalc = false;
//  }

}

void leftMicIsr() {
  last_left_time = left_time;
  left_time = micros();
  recalc = true;
}

void rightMicIsr() {
  last_right_time = right_time;
  right_time = micros();
  recalc = true;
}

// Method stub - calculates angle based on phase shift
float calculateAngle(float phase_shift) {
  return -atan2(phase_shift * SPEED_OF_SOUND, MIC_DIST);
}

void displayAngleOled(double angle) {
  display.print("Angle: ");
  display.print(angle);
  display.drawCircle(display.getCursorX() + 3, display.getCursorY(), 2, 1);
}

void displayLineOled(coord_t start, coord_t end, uint16_t color) {
  display.drawLine(start.x, start.y, end.x, end.y, WHITE);
}
