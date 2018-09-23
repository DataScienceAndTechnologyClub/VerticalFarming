#define PUMP_PIN 8
#define PUMP_SPEED 1.13636

void setupPump() {
  pinMode(PUMP_PIN, OUTPUT);
}

// Problem: This function doesn't return until amount is pumped, which takes a long time. Should be replaced with an interrupt & clock routine.
void pump(double ml) {
  digitalWrite(PUMP_PIN, HIGH);
  delay(round(ml/PUMP_SPEED*1000.0));
  digitalWrite(PUMP_PIN, LOW);
}

void setup() {
  setupPump();
}

void loop() {
  pump(10);
  while (true);
}
