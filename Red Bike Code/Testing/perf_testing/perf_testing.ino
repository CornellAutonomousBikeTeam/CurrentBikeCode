
void setup() {
  Serial.begin(1000000);
}

void loop() {
  unsigned long start;

  start = micros();
  for (unsigned int i = 0; i < 1000000; i++){
    // v1 test code
  }
  Serial.print("v1: ");
  Serial.print  (1.0 / ((micros() - start) / 1000000.0));

  start = micros();
  for (unsigned int i = 0; i < 1000000; i++){
    // v2 test code
  }
  Serial.print(", v2: ");
  Serial.println(1.0 / ((micros() - start) / 1000000.0));
}