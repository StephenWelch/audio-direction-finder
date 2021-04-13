#include <TimerOne.h>
#include <ArducamSSD1306.h>    // Modification of Adafruit_SSD1306 for ESP8266 compatibility
#include <Adafruit_GFX.h>   // Needs a little change in original Adafruit library (See README.txt file)
#include <Wire.h>           // For I2C comm, but needed for not getting compile error

#include <math.h>

#define SAMPLE_PERIOD_US 50
#define BUF_SZ 100

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

typedef struct timestamped_val {
  int val;
  unsigned long ts;
} timestamped_val;

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

  Serial.begin(2000000);
}

timestamped_val left[BUF_SZ];
timestamped_val right[BUF_SZ];
int fill_count;

unsigned long last_print_time, last_sample_time;

void loop() {
  unsigned long curr_ts = micros();

  if(fill_count >= BUF_SZ) {
//      Serial.print("left:");Serial.print(left[i].val);Serial.print(" ");
//      Serial.print("right:");Serial.print(right[i].val);Serial.println();
    
    last_print_time = curr_ts;

    timestamped_val left_peak = {0, 0};
    timestamped_val right_peak = {0, 0};
    int i = 0;
    while(!is_near(left[i].val, v_to_s(5.0), v_to_s(0.05))) {
      i++;
    }
    left_peak = left[i];
    while(!is_near(right[i].val, v_to_s(5.0), v_to_s(0.05))) {
      i++;
    }
    right_peak = right[i];

//    Serial.print("left peak:");
//    Serial.println(left_peak.val);
//    Serial.println(left_peak.ts);
//    Serial.print("right peak:");
//    Serial.println(right_peak.val);
//    Serial.println(right_peak.ts);
      double left_peak_sec = left_peak.ts / 1000000.0;
      double right_peak_sec = right_peak.ts / 1000000.0;
//      unsigned long dt = abs_sub(left_peak.ts, right_peak.ts);
      double dt = right_peak_sec - left_peak_sec;
      Serial.println(dt, 6);
//      double dt_sec = abs_sub(left_peak.ts, right_peak.ts) / 1000000.0;
//    Serial.println(360.0 * 1000.0 * dt_sec, 5);
    
    reset_buffer(left);
    reset_buffer(right);
    
  } else if(curr_ts - last_sample_time >= SAMPLE_PERIOD_US) {
    last_sample_time = curr_ts;
    
    int left_voltage = analogRead(LEFT_MIC);
    int right_voltage = analogRead(RIGHT_MIC);
    
    left[fill_count] = {left_voltage, curr_ts};
    right[fill_count] = {right_voltage, curr_ts};
    fill_count++;
    
//    unsigned long sample_end_ts = micros();
//    Serial.print("sample dt:");
//    Serial.println(sample_end_ts - curr_ts);
  }
  
}

void reset_buffer(timestamped_val *buf) {
  for(int i = 0; i < BUF_SZ; i++) buf[i] = {0, 0};
  fill_count = 0;
}

////////
// Utils
////////
float map_float(float value, float from_low, float from_high, float to_low, float to_high) {
  return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low;
}

float s_to_v(int value) {
  return map_float(value, 0, 1023, 0, 5);
}

int v_to_s(float value) {
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
