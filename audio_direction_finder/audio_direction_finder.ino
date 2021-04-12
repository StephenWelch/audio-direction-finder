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

////////////////////
// TYPE DECLARATIONS
////////////////////

struct timestamped_val {
  unsigned long val;
  unsigned long ts;
};

////////////////////////
// VARIABLE DECLARATIONS
////////////////////////

void setup() {
  // Display initialization
//  display.begin();
//  display.clearDisplay();
//  display.setTextSize(1);
//  display.setTextColor(WHITE);

  // set prescale to 16
  sbi(ADCSRA,ADPS2);
  cbi(ADCSRA,ADPS1);
  cbi(ADCSRA,ADPS0);
  analogReadResolution(7);
  Serial.begin(2000000);
}

volatile timestamped_val left[4];
volatile timestamped_val right[4];
volatile int fill_count;
volatile timestamped_val left_peak, right_peak;
volatile timestamped_val last_left_peak, last_right_peak;

volatile unsigned long last_print_time, last_sample_time;

void loop() {
  unsigned long curr_ts = micros();

//  if(curr_ts - last_print_time >= 200000) {
//    last_print_time = curr_ts;
////    unsigned long dt = abs_sub(left_peak.ts, right_peak.ts);
////    if(left_peak.val != 0 && right_peak.val != 0) {
////      Serial.println(left_peak.ts - last_left_peak.ts);
//////      Serial.println(fmod(360.0 * 1000.0 * abs_sub(left_peak.ts, right_peak.ts) / 1000000.0, 360));
////      last_left_peak.ts = left_peak.ts;
////      last_right_peak.ts = right_peak.ts;
////    }
////    reset_buffer(left);
////    reset_buffer(right);
//  } else 
//  if(curr_ts - last_sample_time >= 25) {
//    last_sample_time = curr_ts;
    unsigned long sample_start = micros();
    int left_voltage = analogRead(LEFT_MIC);
    int right_voltage = analogRead(RIGHT_MIC);
    Serial.println(micros() - sample_start);
//    update_buffer(left, {left_voltage, curr_ts});
//    update_buffer(right, {right_voltage, curr_ts});
//    fill_count++;
//
//    if(fill_count >= 4) {
//      timestamped_val new_left_peak = detect_peak(left);
//      timestamped_val new_right_peak = detect_peak(right);
//      
//      if(new_left_peak.val != 0) left_peak = new_left_peak;
//      if(new_right_peak.val != 0) right_peak = new_right_peak;
//    }
//  }
  
}

void reset_buffer(timestamped_val *buf) {
  for(int i = 0; i < 4; i++) buf[i] = {0, 0};
  fill_count = 0;
}

void update_buffer(timestamped_val *buf, const timestamped_val &sample) {
  buf[3] = buf[2];
  buf[2] = buf[1];
  buf[1] = buf[0];
  buf[0] = sample;
}

timestamped_val detect_peak(timestamped_val *buf) {
  for(int i = 2; i < 4; i++) {
    if(buf[i-2].val < buf[i-1].val && buf[i].val < buf[i-1].val) {
      return buf[i-1];
    }
  }
  return {0, 0};
}

////////
// Utils
////////
float map_float(float value, float from_low, float from_high, float to_low, float to_high) {
  return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low;
}

float map_reading(int value) {
  return map_float(value, 0, 1023, 0, 5);
}

int map_voltage(float value) {
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
