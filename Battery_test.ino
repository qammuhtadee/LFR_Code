const int VOLTAGE_PIN = A0; 

void setup() {
  Serial.begin(9600);
  pinMode(VOLTAGE_PIN, INPUT);
}

void loop() {
  int rawADC = analogRead(VOLTAGE_PIN);
  
  // Calculate the actual voltage hitting the Arduino pin
  float pinVoltage = (rawADC / 1023.0) * 5.0;
  
  // Calculate the total battery voltage (Assuming a perfect 20k/10k divider = 3x scale)
  float batteryVoltage = pinVoltage * 3.0;

  Serial.print("Raw ADC Value: ");
  Serial.print(rawADC);
  
  Serial.print("  |  Pin Voltage: ");
  Serial.print(pinVoltage);
  Serial.print("V");
  
  Serial.print("  |  Est. Battery Voltage: ");
  Serial.print(batteryVoltage);
  Serial.println("V");

  delay(500); // Wait half a second before reading again
}
