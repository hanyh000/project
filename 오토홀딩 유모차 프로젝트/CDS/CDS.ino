void setup() {
  Serial.begin(9600);
}

void loop() {
  delay(1000);

  int lightValue = analogRead(A0);

  Serial.print("lig,");
  Serial.println(lightValue);
}
