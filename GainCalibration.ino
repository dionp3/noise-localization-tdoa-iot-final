//GainCalibration

const int numSensors = 4;
const int sensorPins[numSensors] = {34, 35, 32, 33}; 

const int sampleWindow = 50; 

void setup() {
  Serial.begin(115200);
  Serial.println("Mic1_ADC\tMic2_ADC\tMic3_ADC\tMic4_ADC"); 
}

void loop() {
  unsigned long startMillis = millis();
  
  unsigned int signalMax[numSensors];
  unsigned int signalMin[numSensors];

  // Set nilai awal
  for (int i = 0; i < numSensors; i++) {
    signalMax[i] = 0;
    signalMin[i] = 4095;
  }

  // Sampling data selama 50ms
  while (millis() - startMillis < sampleWindow) {
    for (int i = 0; i < numSensors; i++) {
      unsigned int sample = analogRead(sensorPins[i]);
      
      if (sample < 4095) { 
        if (sample > signalMax[i]) {
          signalMax[i] = sample; 
        } else if (sample < signalMin[i]) {
          signalMin[i] = sample; 
        }
      }
    }
  }

  // Hitung Peak-to-Peak dan Print format Tab-Separated
  for (int i = 0; i < numSensors; i++) {
    unsigned int peakToPeak = signalMax[i] - signalMin[i];
    
    Serial.print(peakToPeak); // Print nilai mentah ADC
    
    // Pemisah tab antar kolom
    if (i < numSensors - 1) {
      Serial.print("\t");
    }
  }
  
  Serial.println(); 
  
  delay(5); 
}
