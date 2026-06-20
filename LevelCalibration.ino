#include <math.h> 

const int numSensors = 4;
const int sensorPins[numSensors] = {34, 35, 32, 33}; 
const char* labels[numSensors] = {"Depan", "Kiri", "Belakang", "Kanan"};

const int sampleWindow = 50;            
const float calibrationOffset = 79.0;   

const float rawSensorPoints[] = {43.60, 45.12, 46.86, 49.61, 55.79, 64.99}; 
const float trueSLMPoints[]   = {45.0, 51.0, 55.5, 60.5, 71.0, 80.0};
const int numPoints = 6;

void setup() {
  Serial.begin(115200);
}

void loop() {
  unsigned long startMillis = millis();
  unsigned int signalMax[numSensors];
  unsigned int signalMin[numSensors];

  for (int i = 0; i < numSensors; i++) {
    signalMax[i] = 0;
    signalMin[i] = 4095; 
  }

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

  for (int i = 0; i < numSensors; i++) {
    unsigned int peakToPeak = signalMax[i] - signalMin[i];
    double volts = (peakToPeak * 3.3) / 4096.0; 
    double vRms = volts * 0.3535; 
    double rawDB = 0;
    if (vRms > 0.001) { 
      rawDB = 20.0 * log10(vRms) + calibrationOffset;
    }
    double trueDB = interpolateDB(rawDB);
    if (trueDB < 0) {
      trueDB = 0; 
    }
    Serial.print(trueDB, 2); 
    if (i < numSensors - 1) {
      Serial.print("\t");
    }
  }
  Serial.println(); 
  delay(500); 
}

float interpolateDB(float rawDB) {
  if (rawDB <= rawSensorPoints[0]) {
    return trueSLMPoints[0] - (rawSensorPoints[0] - rawDB);
  }
  if (rawDB >= rawSensorPoints[numPoints - 1]) {
    return trueSLMPoints[numPoints - 1] + (rawDB - rawSensorPoints[numPoints - 1]);
  }
  for (int i = 0; i < numPoints - 1; i++) {
    if (rawDB >= rawSensorPoints[i] && rawDB <= rawSensorPoints[i+1]) {
      return trueSLMPoints[i] + ((rawDB - rawSensorPoints[i]) * (trueSLMPoints[i+1] - trueSLMPoints[i]) / (rawSensorPoints[i+1] - rawSensorPoints[i]));
    }
  }
  return rawDB;
}
