void setup() {
    Serial.begin(9600);

   for (int i=0; i < 10; i++){
     Serial.println("Hello world!");
   }
}

void loop() {
   Serial.println("Echo");
   delay(100);
}
