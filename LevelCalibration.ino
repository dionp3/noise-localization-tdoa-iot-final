//LevelCalibration
//Censor DB With SLM Calibration

#include <math.h> 

// ================= KONFIGURASI SENSOR =================
const int numSensors = 4;
const int sensorPins[numSensors] = {34, 35, 32, 33}; 
const char* labels[numSensors] = {"Depan", "Kiri", "Belakang", "Kanan"};

// ================= PARAMETER SISTEM =================
const int sampleWindow = 50;            // Durasi sampling (ms)
const float calibrationOffset = 79.0;   // Offset awal untuk konversi dB

// ================= DATA KALIBRASI =================
// Digunakan untuk interpolasi antara nilai sensor dan nilai SLM (Sound Level Meter)
const float rawSensorPoints[] = {43.60, 45.12, 46.86, 49.61, 55.79, 64.99}; 
const float trueSLMPoints[]   = {45.0, 51.0, 55.5, 60.5, 71.0, 80.0};
const int numPoints = 6;
// =====================================================

void setup() {
  Serial.begin(115200);
}

void loop() {
  unsigned long startMillis = millis();
  
  unsigned int signalMax[numSensors];
  unsigned int signalMin[numSensors];

  // Inisialisasi nilai awal
  for (int i = 0; i < numSensors; i++) {
    signalMax[i] = 0;
    signalMin[i] = 4095; 
  }

  // ================= PROSES SAMPLING =================
  // Mengambil nilai maksimum dan minimum dari tiap sensor
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

  // ================= PERHITUNGAN dB =================
  for (int i = 0; i < numSensors; i++) {

    // 1. Hitung amplitudo (peak-to-peak)
    unsigned int peakToPeak = signalMax[i] - signalMin[i];
    
    // 2. Konversi ke tegangan (Volt)
    double volts = (peakToPeak * 3.3) / 4096.0; 
    
    // 3. Hitung nilai RMS (Root Mean Square)
    double vRms = volts * 0.3535; 
    
    // 4. Hitung nilai dB mentah
    double rawDB = 0;
    if (vRms > 0.001) { 
      rawDB = 20.0 * log10(vRms) + calibrationOffset;
    }

    // 5. Koreksi menggunakan interpolasi kalibrasi
    double trueDB = interpolateDB(rawDB);

    // 6. Batasi nilai minimum (tidak boleh negatif)
    if (trueDB < 0) {
      trueDB = 0; 
    }

    // ================= OUTPUT SERIAL =================
    Serial.print(trueDB, 2); // Tampilkan 2 angka desimal
    
    // Separator antar sensor
    if (i < numSensors - 1) {
      Serial.print("\t");
    }
  }
  
  Serial.println(); 
  delay(1000); // Delay agar mudah dibaca saat monitoring
}

// =====================================================
// FUNGSI INTERPOLASI KALIBRASI
// Mengubah nilai dB mentah menjadi nilai mendekati SLM
// =====================================================
float interpolateDB(float rawDB) {

  // Jika di bawah batas minimum kalibrasi
  if (rawDB <= rawSensorPoints[0]) {
    return trueSLMPoints[0] - (rawSensorPoints[0] - rawDB);
  }

  // Jika di atas batas maksimum kalibrasi
  if (rawDB >= rawSensorPoints[numPoints - 1]) {
    return trueSLMPoints[numPoints - 1] + (rawDB - rawSensorPoints[numPoints - 1]);
  }

  // Interpolasi linear antar titik kalibrasi
  for (int i = 0; i < numPoints - 1; i++) {
    if (rawDB >= rawSensorPoints[i] && rawDB <= rawSensorPoints[i+1]) {
      return trueSLMPoints[i] +
        ((rawDB - rawSensorPoints[i]) *
        (trueSLMPoints[i+1] - trueSLMPoints[i]) /
        (rawSensorPoints[i+1] - rawSensorPoints[i]));
    }
  }

  // Fallback jika tidak masuk kondisi di atas
  return rawDB;
}
