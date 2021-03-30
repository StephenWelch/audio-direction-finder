#include <TimerOne.h>

const int SAMPLE_PERIOD_US = 10000;
const int LEFT_MIC = A0;
const int RIGHT_MIC = A1;
const float threshold_voltage = 0; // Placeholder value - depends on final circuit design

// Store past voltage values so we can detect when a measurement has crossed a threshold
float last_left_voltage, last_right_voltage;
// For measuring phase shift
unsigned long left_time, right_time;
// For profiling execution times
unsigned long curr_ts, last_ts, latest_ts_delta;

// Interpolates a parameter based on its possible range of values and a desired range of values
// This is a float-based analogue to the map() function provided by the Arduino standard library
float map_float(float value, float from_low, float from_high, float to_low, float to_high) {
  return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low;
}

void setup() {
  // Disable interrupts while we perform hardware setup
  noInterrupts();
  Timer1.initialize(SAMPLE_PERIOD_US);
  // Attaches a function to be called at the configured interval
  Timer1.attachInterrupt(testSample);

  // put your setup code here, to run once:
  Serial.begin(115200);

  // Setup is finished - reenable interrupts
  interrupts();
}

void loop() {
  // Empty for now - non-time-sensitive processing will eventually go here
}

void samplePhaseShift() {
  curr_ts = micros();

  // Read voltage from analog inputs and convert from integer value to 0-5V
  float left_voltage = map_float(analogRead(LEFT_MIC), 0, 1023, 0, 5);
  float right_voltage = map_float(analogRead(RIGHT_MIC), 0, 1023, 0, 5);

  // Note the time at which the signal was sampled is used instead of micros()
  if (left_voltage < threshold_voltage && !(last_left_voltage < threshold_voltage)) {
    left_time = curr_ts;
  }

  if (right_voltage < threshold_voltage && !(last_right_voltage < threshold_voltage)) {
    right_time = curr_ts;
  }
  
  float phase_shift_sec = abs(left_time - right_time) / 1000000.0;

  last_left_voltage = left_voltage;
  last_right_voltage = right_voltage;
  last_ts = curr_ts;
}

void testSample()
{
  curr_ts = micros();
  int left_voltage_raw = analogRead(LEFT_MIC);
  float left_voltage = map_float(left_voltage_raw, 0, 1023, 0, 5);
  //  latest_voltage = left_voltage;
  latest_ts_delta = curr_ts - last_ts;

  // For testing purposes only
  Serial.print(left_voltage);
  Serial.print(" ");
  Serial.print(latest_ts_delta);
  Serial.println();
  last_ts = curr_ts;
}
