#include <TimerOne.h>
#include <ArducamSSD1306.h>    // Modification of Adafruit_SSD1306 for ESP8266 compatibility
#include <Adafruit_GFX.h>   // Needs a little change in original Adafruit library (See README.txt file)
#include <Wire.h>           // For I2C comm, but needed for not getting compile error

#include <math.h>

#define SAMPLE_PERIOD_US 50
#define BUF_SIZE 20

const int OLED_RESET_PIN = 16;
const int OLED_ADDR = 0x78;
const int LEFT_MIC = A0;
const int RIGHT_MIC = A1;
const unsigned long SPEED_OF_SOUND = 343 / pow(10, 9); // Speed of sound in microseconds
const double MIC_DIST = 1.0; // Distance between microphones in meters
const float THRESH_VOLTAGE = 0; // Placeholder value - depends on final circuit design

ArducamSSD1306 display(OLED_RESET_PIN);

// Variables shared between ISRs and the main loop go here
// These must be marked volatile so that they aren't optimized out by the compiler
volatile unsigned long left_time, right_time, last_left_time, last_right_time, phase_shift;
volatile bool recalc = false;
volatile float last_left_voltage, last_right_voltage;
volatile unsigned long curr_ts, last_ts, latest_ts_delta;

struct coord {
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
  
};

struct timestamped_val {
  float val;
  unsigned long ts;
};

template<typename T>
struct circular_buf {
  T buf[BUF_SIZE];
  int headIdx;
  int tailIdx;

  T getHead() {
    return buf[headIdx];
  }

  T getTail() {
    return buf[tailIdx];
  }

  void push(T val) {
    buf[tailIdx] = val;
    tailIdx = (tailIdx + 1) % BUF_SIZE;
    if(tailIdx == headIdx) headIdx++;
  }

  float pop() {
    float val = buf[headIdx];
    int newHeadIdx = (headIdx + 1) % BUF_SIZE;
    if(newHeadIdx != tailIdx) newHeadIdx++;
    return val;
  }
};

float map_float(float value, float from_low, float from_high, float to_low, float to_high) {
  return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low;
}

// Arrow graphic
coord line_start = {64, 32};
coord line_end = {64, 64};
coord arrow_head_l_end = line_start + (coord){-8, 8};
coord arrow_head_r_end = line_start + (coord){8, 8};

coord origin = {64, 48};

// New buffer impl
volatile circular_buf<timestamped_val> left_buf, right_buf;

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
  Timer1.initialize(SAMPLE_PERIOD_US);
  // Attaches a function to be called at the configured interval
  Timer1.attachInterrupt(sampleIsr);
  
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

void sampleIsr() {
  curr_ts = micros();

  // Read voltage from analog inputs and convert from integer value to 0-5V
  float left_voltage = map_float(analogRead(LEFT_MIC), 0, 1023, 0, 5);
  float right_voltage = map_float(analogRead(RIGHT_MIC), 0, 1023, 0, 5);

  // Note the time at which the signal was sampled is used instead of micros()
  if (left_voltage < THRESH_VOLTAGE && !(last_left_voltage < THRESH_VOLTAGE)) {
    left_time = curr_ts;
  }

  if (right_voltage < THRESH_VOLTAGE && !(last_right_voltage < THRESH_VOLTAGE)) {
    right_time = curr_ts;
  }
  
  float phase_shift_sec = abs(left_time - right_time) / 1000000.0;

  last_left_voltage = left_voltage;
  last_right_voltage = right_voltage;
  last_ts = curr_ts;
}

void sampleIsrWithBuf() {
  curr_ts = micros();

  // Read voltage from analog inputs and convert from integer value to 0-5V
  float left_voltage = map_float(analogRead(LEFT_MIC), 0, 1023, 0, 5);
  float right_voltage = map_float(analogRead(RIGHT_MIC), 0, 1023, 0, 5);

  left_buf.push({left_voltage, curr_ts});
  right_buf.push({right_voltage, curr_ts});

  last_left_voltage = left_voltage;
  last_right_voltage = right_voltage;
  last_ts = curr_ts;
}

// Method stub - calculates angle based on phase shift
float calculateAngle(float phase_shift) {
  return 0.0;
}

void displayAngleOled(double angle) {
  display.print("Angle: ");
  display.print(angle);
  display.drawCircle(display.getCursorX() + 3, display.getCursorY(), 2, 1);
}

void displayLineOled(coord start, coord end, uint16_t color) {
  display.drawLine(start.x, start.y, end.x, end.y, WHITE);
}
