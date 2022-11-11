
int ledPin = 13;
void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600)
}

void loop() {
  for (int i = 0; i < 5; i++) {
    Serial.println("Blinking at 0.5 times per second");
    digitalWrite(ledPin, HIGH);
    delay(1000);
    digitalWrite(ledPin, LOW);
    delay(1000);
  }
  
  for (int i = 0; i < 10; i++) {
    Serial.println("Blinking at 1 times per second");
    digitalWrite(ledPin, HIGH);
    delay(500);
    digitalWrite(ledPin, LOW);
    delay(500);
  }
}
