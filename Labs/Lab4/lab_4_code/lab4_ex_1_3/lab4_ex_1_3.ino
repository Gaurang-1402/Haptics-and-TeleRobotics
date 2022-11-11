void setup() {
  Serial.begin(9600);
  pinMode(12, INPUT_PULLUP);
}

void loop() {

   int sensorVal = digitalRead(12);

   if (sensorVal == 0) {
     Serial.println("ON!");
   } else if (sensorVal == 1) {
      Serial.println("OFF!");
   }
   
   delay(1000);
}
