#include <math.h>

const int LEFT_MIC = 2;
const int RIGHT_MIC = 3;
const unsigned long SPEED_OF_SOUND = 343 / pow(10, 9); // Speed of sound in microseconds
const double MIC_DIST = 1.0; // Distance between microphones in meters

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
  // Disable interrupts while we perform hardware setup
  noInterrupts();
  // Setup interrupts to trigger on rising edge of input signal
  attachInterrupt(digitalPinToInterrupt(LEFT_MIC), leftMicIsr, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_MIC), rightMicIsr, RISING);

  // put your setup code here, to run once:
  Serial.begin(115200);

  // Setup is finished - reenable interrupts
  interrupts();
}

void loop() {
  // Empty for now - non-time-sensitive processing will eventually go here
  float phase_shift = abs(left_time - right_time);
  float calculated_angle = calculateAngle(phase_shift);
  Serial.print("Phase shift:");
  Serial.print(phase_shift);
  Serial.print("Calculated Angle:");
  Serial.print(calculated_angle);
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