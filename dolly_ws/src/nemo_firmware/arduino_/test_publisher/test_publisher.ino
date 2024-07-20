void setup() {
  Serial.begin(115200);

}

void loop() {
  if(Serial.available()) {
    int x = Serial.readString().toInt();
      Serial.print(x);
    }
  }
