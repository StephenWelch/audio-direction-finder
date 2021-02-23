const int LEFT_MIC = A0;
const int RIGHT_MIC = A1;
const float threshold_value = 0;
float last_left_voltage = 0;
float last_right_voltage = 0;
long left_time = 0;
long right_time = 0;

float map_float(float value, float from_low, float from_high, float to_low, float to_high) {
  return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  int left_value_raw = analogRead(LEFT_MIC);
  int right_value_raw = analogRead(RIGHT_MIC);
  float left_voltage = map_float(left_value_raw, 0, 1023, 0, 5);
  float right_voltage = map_float(right_value_raw, 0, 1023, 0, 5);

  if(left_voltage < threshold_voltage && !(last_left_voltage < threshold_voltage)) {
    left_time = millis();
  }

  if(right_voltage < threshold_voltage && !(last_right_voltage < threshold_voltage)) {
    right_time = millis();
  }

  float phase_shift_sec = abs(left_time - right_time) / 1000.0;

  Serial.print(left_voltage);
  Serial.print(" ");
  Serial.print(right_voltage);
  Serial.print(" ");
  Serial.print(phase_shift_sec);

  last_left_voltage = left_voltage;
  last_right_voltage = right_voltage;
}
