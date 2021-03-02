#include <TimerOne.h>

const int LEFT_MIC = A0;
const int RIGHT_MIC = A1;
const float threshold_voltage = 0;
float last_left_voltage = 0;
float last_right_voltage = 0;
long left_time = 0;
long right_time = 0;
unsigned long curr_ts = 0;
unsigned long last_ts = 0;
unsigned long latest_ts_delta = 0;
const int VOLTAGE_RAW_BUF_SIZE = 512;
float latest_voltage = 0;

float map_float(float value, float from_low, float from_high, float to_low, float to_high) {
  return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low;
}

void setup() {
  noInterrupts();
  Timer1.initialize(10000);
  Timer1.attachInterrupt(sample);

  // put your setup code here, to run once:
  Serial.begin(115200);
  
  interrupts();
}

void loop() {
  //  curr_ts = micros();

  // put your main code here, to run repeatedly:
  //  int left_voltage_raw = analogRead(LEFT_MIC);
  //  int right_voltage_raw = analogRead(RIGHT_MIC);
  //  float left_voltage = map_float(left_voltage_raw, 0, 1023, 0, 5);
  //  float right_voltage = map_float(right_voltage_raw, 0, 1023, 0, 5);
  //
  //  if(left_voltage < threshold_voltage && !(last_left_voltage < threshold_voltage)) {
  //    left_time = curr_ts;
  //  }
  //
  //  if(right_voltage < threshold_voltage && !(last_right_voltage < threshold_voltage)) {
  //    right_time = curr_ts;
  //  }
  //
  //  float phase_shift_sec = abs(left_time - right_time) / 1000.0;

  //  Serial.print(left_voltage);
  //  Serial.print(" ");
  //  Serial.print(curr_ts - last_ts);
  //  Serial.println();

  //  last_left_voltage = left_voltage;
  //  last_right_voltage = right_voltage;
  //  last_ts = curr_ts;

//  Serial.print(latest_voltage);
//  Serial.print(" ");
//  Serial.print(latest_ts_delta);
//  Serial.println();
}

void sample()
{
  curr_ts = micros();
  int left_voltage_raw = analogRead(LEFT_MIC);
  float left_voltage = map_float(left_voltage_raw, 0, 1023, 0, 5);
//  latest_voltage = left_voltage;
  latest_ts_delta = curr_ts - last_ts;

  // For testing purposes only
  Serial.print(left_voltage);
//  Serial.print(" ");
//  Serial.print(latest_ts_delta);
  Serial.println();
  last_ts = curr_ts;
}
