const int LEFT_MIC = A0;
const int RIGHT_MIC = A1;
const float threshold_voltage = 0;
float last_left_voltage = 0;
float last_right_voltage = 0;
long left_time = 0;
long right_time = 0;
unsigned long curr_ts = 0;
unsigned long last_ts = 0;
float voltage_buf[512];
float *voltage_buf_ptr = voltage_buf;

float map_float(float value, float from_low, float from_high, float to_low, float to_high) {
  return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low;
}

void setup() {
  noInterrupts();
  // Reset timer registers
  TCCR1A = 0;
  TCCR1B = 0;
  // Reset counter register
  TCNT1  = 0;
  // Set comparison register to (16 Mhz) / (freq * prescaler) - 1
  OCR1A = 124;// = (16*10^6) / (2000*64) - 1 (must be <256)
  // Set CTC (Clear timer on compare) mode
  TCCR1A |= (1 << WGM01);
  // Set prescaler
  TCCR1B |= (1 << CS01) | (1 << CS00);   
  // Enable timer compare interrupt
  TIMSK1 |= (1 << OCIE0A);
  
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
}

ISR(TIMER1_COMPA_vect) 
{
//  curr_ts = micros();
  int left_voltage_raw = analogRead(LEFT_MIC);
  float left_voltage = map_float(left_voltage_raw, 0, 1023, 0, 5);

  Serial.print(left_voltage);
  Serial.print(" ");
//  Serial.print(curr_ts - last_ts);
  Serial.println();

  last_left_voltage = left_voltage;
  last_ts = curr_ts;
}
