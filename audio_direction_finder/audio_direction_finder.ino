#include <TimerOne.h>
#include <ArducamSSD1306.h>    // Modification of Adafruit_SSD1306 for ESP8266 compatibility
#include <Adafruit_GFX.h>   // Needs a little change in original Adafruit library (See README.txt file)
#include <Wire.h>           // For I2C comm, but needed for not getting compile error

#include <math.h>

#define SAMPLE_PERIOD_US 450
#define BUF_SIZE 10

// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

////////////
// CONSTANTS
////////////
const int OLED_RESET_PIN = 16;
const int OLED_ADDR = 0x78;
const int LEFT_MIC = A0;
const int RIGHT_MIC = A1;
const unsigned long SPEED_OF_SOUND = 343 / pow(10, 9); // Speed of sound in microseconds
const double MIC_DIST = 1.0; // Distance between microphones in meters
const float THRESH_VOLTAGE = 0; // Placeholder value - depends on final circuit design

////////////////////
// TYPE DECLARATIONS
////////////////////
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
  unsigned long val;
  unsigned long ts;
};

template<typename T>
struct circular_buf {
  T buf[BUF_SIZE];
  T null_val;
  int headIdx = 0;
  int tailIdx = 0;

  void push(T val) {
    buf[tailIdx] = val;
    if(!full())tailIdx = (tailIdx + 1) % BUF_SIZE;
  }

  T pop() {
    T val = null_val;
    if(!empty()) {
      val = buf[headIdx];
      headIdx = (headIdx + 1) % BUF_SIZE;
    }
    return val;
  }

  T getTail() {
    if(!empty()) {
      return buf[(tailIdx - 1) % BUF_SIZE];
    }
    return null_val;
  }

  T getAt(int idx) {
    return buf[(headIdx + idx) % BUF_SIZE];
  }

  bool full() {
    return (tailIdx + 1) % BUF_SIZE == headIdx;
  }

  bool empty() {
    return headIdx == tailIdx;
  }

  int len() {
    if (tailIdx >= headIdx) return tailIdx - headIdx;
    return BUF_SIZE - headIdx - tailIdx;
  }
  
};

////////////////////////
// VARIABLE DECLARATIONS
////////////////////////
//ArducamSSD1306 display(OLED_RESET_PIN);

// Variables shared between ISRs and the main loop go here
// These must be marked volatile so that they aren't optimized out by the compiler
volatile unsigned long last_left_peak, last_right_peak;
volatile timestamped_val last_left, last_right;
volatile bool recalc = false;
volatile unsigned long curr_ts, last_ts, latest_ts_delta;
volatile unsigned long leftRising, leftFalling, rightRising, rightFalling, leftPeak, rightPeak;
volatile unsigned long lastDumpTime;
volatile bool leftReady, rightReady;
// Arrow graphic
coord line_start = {64, 32};
coord line_end = {64, 64};
coord arrow_head_l_end = line_start + (coord){-8, 8};
coord arrow_head_r_end = line_start + (coord){8, 8};

coord origin = {64, 48};

volatile circular_buf<timestamped_val> left_buf, right_buf;

void setup() {
  // Display initialization
//  display.begin();
//  display.clearDisplay();
//  display.setTextSize(1);
//  display.setTextColor(WHITE);

  // set prescale to 16
  sbi(ADCSRA,ADPS2) ;
  cbi(ADCSRA,ADPS1) ;
  cbi(ADCSRA,ADPS0) ;

  Serial.begin(2000000);
  
  // Disable interrupts while we perform hardware setup
  noInterrupts();
  // Setup interrupts to trigger on periodic timer
  Timer1.initialize(SAMPLE_PERIOD_US);
  // Attaches a function to be called at the configured interval
  Timer1.attachInterrupt(sampleIsr);
  // Setup is finished - reenable interrupts
  interrupts();
}

void loop() {
  // Empty for now - non-time-sensitive processing will eventually go here
//  Serial.println(360.0 * ((float)abs(leftPeak - rightPeak) / 1000000.0) * 1000.0);

//  float calculated_angle = calculateAngle(phase_shift);
//  float calculated_angle = millis() / 250;  
//  display.clearDisplay();
//  display.setCursor(20,20);
//  displayAngleOled(calculated_angle);
//  display.setCursor(64, 64);

//  float calculated_angle_rad = calculated_angle * PI / 180.0;
//  displayLineOled(line_start.rotate(origin, calculated_angle_rad), line_end.rotate(origin, calculated_angle_rad), WHITE);
//  displayLineOled(line_start.rotate(origin, calculated_angle_rad), arrow_head_l_end.rotate(origin, calculated_angle_rad), WHITE);
//  displayLineOled(line_start.rotate(origin, calculated_angle_rad), arrow_head_r_end.rotate(origin, calculated_angle_rad), WHITE);
//  display.display();
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
  bool dump = curr_ts - lastDumpTime >= 200000;
  bool dumped = false;

  // Read voltage from analog inputs and convert from integer value to 0-5V
  int left_voltage = analogRead(LEFT_MIC);
  int right_voltage = analogRead(RIGHT_MIC);

  if(last_left.val != 0) {
    if(last_left.val < 716 && left_voltage > 716) {
      leftRising = (curr_ts + last_left.ts) / 2;
    }
  
    if(last_left.val > 716 && left_voltage < 716 && leftRising != 0) {
      leftFalling = (curr_ts + last_left.ts) / 2;
  
      leftPeak = (leftRising + leftFalling) / 2.0;
      if(dump) {
        dumped = true;
        if(last_left_peak != 0) {
          Serial.print("left_per:"); Serial.println(1.0 / ((float)(leftPeak - last_left_peak) / 1000000.0)); 
  //          Serial.println(fmod(360.0 * (abs_sub(leftPeak, rightPeak) / 1000000.0) * 1000.0, 360.0)); 
  //          Serial.println(abs_sub(leftPeak, rightPeak));
        }
        last_left_peak = 0;
        last_left = {0, 0};
      } else {
        last_left_peak = leftPeak;
      }
      leftRising = 0;
      leftReady = true;
    }
  }
  
  
  if(last_right.val != 0) {
    if(last_right.val < 716 && right_voltage > 716) {
      rightRising = (curr_ts + last_right.ts) / 2;
    }
  
    if(last_right.val > 716 && right_voltage < 716 && rightRising != 0) {
      rightFalling = (curr_ts + last_right.ts) / 2;
  
      rightPeak = (rightRising + rightFalling) / 2.0;
      if(dump) {
        dumped = true;
        if(last_right_peak != 0) {
          Serial.print("right_per:"); Serial.println(1.0 / ((float)(rightPeak - last_right_peak) / 1000000.0)); 
  //          Serial.println(fmod(360.0 * (abs_sub(leftPeak, rightPeak) / 1000000.0) * 1000.0, 360));
  //          Serial.println(abs_sub(leftPeak, rightPeak));
        }
        last_right_peak = 0;
        last_right = {0, 0};
      } else {
        last_right_peak = rightPeak;
      }
      rightRising = 0;
      rightReady = true;
    }
  }
  

  if(dumped) {
    lastDumpTime = curr_ts;
  }

  last_left = {left_voltage, curr_ts};
  last_right = {right_voltage, curr_ts};

//  unsigned long diff = 0;
//  if(leftPeak != 0 && rightPeak != 0) {
//    if(leftPeak > rightPeak) {
//      diff = leftPeak - rightPeak;
//    } else {
//      diff = rightPeak - leftPeak;
//    }
//  //    Serial.print("Phase shift: "); 
//    Serial.println(diff);
//  }
//
  
  // Plot voltage measurements 
//  Serial.print("left:"); Serial.print(map_float(left_voltage, 0, 1023, 0, 5)); Serial.print(" ");
//  Serial.print("right:"); Serial.print(map_float(right_voltage, 0, 1023, 0, 5)); Serial.println();
  
  last_ts = curr_ts;
}

// Method stub - calculates angle based on phase shift
float calculateAngle(float phase_shift) {
  return 0.0;
}

void displayAngleOled(double angle) {
//  display.print("Angle: ");
//  display.print(angle);
//  display.drawCircle(display.getCursorX() + 3, display.getCursorY(), 2, 1);
}

void displayLineOled(coord start, coord end, uint16_t color) {
//  display.drawLine(start.x, start.y, end.x, end.y, WHITE);
}

////////
// Utils
////////
constexpr float map_float(float value, float from_low, float from_high, float to_low, float to_high) {
  return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low;
}

constexpr float map_reading(int value) {
  return map_float(value, 0, 1023, 0, 5);
}

constexpr int map_voltage(float value) {
  return map_float(value, 0, 5, 0, 1023);
}

bool is_near(float a, float b, float thresh) {
  return abs(a - b) <= thresh;
}

unsigned long abs_sub(unsigned long a, unsigned long b) {
  if(a > b) {
    return a - b;
  } else {
    return b - a;
  }
}
