//RawDataCensor

const int micPins[4] = {34, 35, 32, 33};

void setup() {
  Serial.begin(115200);
}

void loop() {
  int raw1 = analogRead(micPins[0]);
  int raw2 = analogRead(micPins[1]);
  int raw3 = analogRead(micPins[2]);
  int raw4 = analogRead(micPins[3]);

  Serial.print(raw1);
  Serial.print(",");
  Serial.print(raw2);
  Serial.print(",");
  Serial.print(raw3);
  Serial.print(",");
  Serial.println(raw4);

  delay(5); 
}
