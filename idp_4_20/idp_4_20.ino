#include <TimerOne.h>
#include <ArducamSSD1306.h>    // Modification of Adafruit_SSD1306 for ESP8266 compatibility
#include <Adafruit_GFX.h>   // Needs a little change in original Adafruit library (See README.txt file)
#include <Wire.h>           // For I2C comm, but needed for not getting compile error

#include <math.h>

#define SAMPLE_PERIOD_US 10
#define BUF_SZ 100
#define NUM_AVE 30

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
const double MIC_DIST_M = 0.06;
const double SPEED_OF_SOUND = 343; // Speed of sound in microseconds
const double MAX_PHASE_SHIFT = MIC_DIST_M / SPEED_OF_SOUND;
ArducamSSD1306 display(OLED_RESET_PIN);

////////////////////
// TYPE DECLARATIONS
////////////////////

typedef struct timestamped_val {
  int val;
  unsigned long ts;
} timestamped_val;

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

  Serial.begin(9600);

  display.begin();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(20,20);

  display.print("Intializing");
  display.display();
}

timestamped_val left[BUF_SZ];
timestamped_val right[BUF_SZ];
int fill_count;

double rollingAverage = 90;

double expAverage = 90;
double prevExp = 90;
double alpha = 0.5;

unsigned long last_print_time, last_sample_time;

bool debug = 0;
bool hold  = 1;

int usPerRead = 36;
double usPerSec = 1000000;

int indOffset = 0;

coord_t line_start = {64, 32};
coord_t line_end = {64, 64};
coord_t arrow_head_l_end = line_start + (coord_t){-8, 8};
coord_t arrow_head_r_end = line_start + (coord_t){8, 8};
coord_t origin = {64, 48};

void loop() {

  // Get current Time
  unsigned long curr_ts = micros();

  if(fill_count >= BUF_SZ) {

    // Init values for peaks
    timestamped_val left_peak = {0, 0};
    timestamped_val right_peak = {0, 0};
    
    int i = 10;
    while(!isPeak(i, left) && (i < BUF_SZ)){
      i++;
    }
    left_peak = left[i];
    
    int k = 10;
    while(!isPeak(k, right) && (k < BUF_SZ)){
      k++;
    }
    right_peak = right[k];

    if (debug){
      Serial.println("---------------");
      for(int j = 1; j < BUF_SZ; j++){
        Serial.print(" "); Serial.print(j);
        Serial.print(" "); Serial.print(left[j].val);
        Serial.print(" "); Serial.print(right[j].val);
        //Serial.print(" "); Serial.print(left[j].ts);
        //Serial.print(" "); Serial.print(right[j].ts);
        //Serial.print(" "); Serial.print(right[j].ts - right[j-1].ts);
        Serial.println(" ");
      }
      Serial.println("---------------");
      Serial.print("i: "); Serial.println(i);
      Serial.print("k: "); Serial.println(k);
      Serial.println("---------------");

      display.clearDisplay();
      display.setCursor(20,20);
      displayAngleOled(calculated_angle);
      display.setCursor(64, 64);
     
      while(hold){
        delay(1000);
      }
      
    }
    else {

      // Offset i by hardware delay
      i = i-indOffset;
      
      // Get difference
      int diff = (k>=i) ? (k-i) : (i-k);

      // If difference is greater than the period, subtract one period.
      diff = (diff>24) ? (diff-25) : diff;

      // If the difference is greater than half the period, subtract 1 period
      diff = (diff > 12) ? (25-diff) : diff;
      
      // Set delta
      double dt = diff*usPerRead;
      
      // Convert from us to s
      dt = dt/usPerSec;
      double angle = calculate_angle(dt);

      if (angle != -1) {
      
      // Account for direction of angle
      angle = (k>i) ? 180-angle : angle;

      // Add angle to rolling average
      rollingAverage += angle;
      rollingAverage = rollingAverage/2;

      // Add angle to exponential average
      expAverage = (alpha*angle)+((1-alpha)*prevExp);
      prevExp = angle;
      
      //Serial.print("i: "); Serial.print(i); Serial.print(" ");
      //Serial.print("k: "); Serial.print(k); Serial.print(" ");
      //Serial.print("diff: ");Serial.print(diff);Serial.print(" ");
      //Serial.print("dt: ");Serial.print(dt, 6);Serial.print(" ");
      //Serial.print("dist: ");Serial.print(dt * SPEED_OF_SOUND);Serial.print(" ");
      Serial.print("angle: ");Serial.print(angle); Serial.print(" ");
      Serial.print("Rolling: ");Serial.print(rollingAverage); Serial.print(" ");
      Serial.print("Exponential: ");Serial.print(expAverage); Serial.print(" ");
      Serial.println();

      }
    
    }

    // Reset buffers
    reset_buffer(left);
    reset_buffer(right);
    
  } // take a sample of both waves
  else {
    last_sample_time = curr_ts;
    
    int left_voltage = analogRead(LEFT_MIC);
    int right_voltage = analogRead(RIGHT_MIC);
    
    left[fill_count] = {left_voltage, curr_ts};
    right[fill_count] = {right_voltage, curr_ts};
    fill_count++;
    
  }
  
  
}

bool isPeak(int idx, timestamped_val arr[BUF_SZ]){

  int count = 0;
  int range = 2;

  // Iterate for the +-5 terms
  for (int i = -range; i <= range; i++){

    // Check for a valid term
    if (i!=0 && idx+i >= 0 && idx+i < BUF_SZ) {

      // Check whether that term is less than a certain peak
      if (arr[idx+i].val <= arr[idx].val){
        count++;
      }
    }
    
  }

  if (count > 0.9*range*2){
    return true;
  }

  return false;
  
}

void reset_buffer(timestamped_val *buf) {
  for(int i = 0; i < BUF_SZ; i++) buf[i] = {0, 0};
  fill_count = 0;
}

////////
// Utils
////////
float calculate_angle(double phase_shift_secs) {
  double val = phase_shift_secs * SPEED_OF_SOUND / MIC_DIST_M;
  if (val <= 1 && val >= -1){
    return degrees(acos(val));
  }
  return -1;
}

double map_float(double value, double from_low, double from_high, double to_low, double to_high) {
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

void displayAngleOled(double angle) {
  display.print("Angle: ");
  display.print(angle);
  display.drawCircle(display.getCursorX() + 3, display.getCursorY(), 2, 1);
}

void displayLineOled(coord_t start, coord_t end, uint16_t color) {
  display.drawLine(start.x, start.y, end.x, end.y, WHITE);
}
