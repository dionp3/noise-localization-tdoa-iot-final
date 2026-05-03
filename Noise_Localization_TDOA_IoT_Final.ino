#include <WiFi.h>
#include <math.h>
#include <PubSubClient.h>

// ================= WIFI & MQTT =================
const char* ssid = "NamaWIFI?";
const char* password = "PasswordWIFI?";
const char* mqtt_server = "ServerMQTTAddress?"; 
const char* TOKEN = "TokenDeviceThingsboard?"; 

WiFiClient espClient;
PubSubClient client(espClient);

// ================= SENSOR =================
const int numSensors = 4;
// Urutan: 0=Depan, 1=Kiri, 2=Belakang, 3=Kanan
const int sensorPins[numSensors] = {34, 35, 32, 33};

// ================= PARAMETER =================
const int sampleWindow = 50;
const float dbThreshold = 55.0; 
const float calibrationOffset = 79.0;

// ================= DATA KALIBRASI =================
const float rawSensorPoints[] = {43.60, 45.12, 46.86, 49.61, 55.79, 64.99}; 
const float trueSLMPoints[]   = {45.0, 51.0, 55.5, 60.5, 71.0, 80.0};
const int numPoints = 6;

// ================= VARIABEL LOCK TDOA =================
bool isSoundLocked = false; 
float lockedAzimuth = -1;
String lockedArah = "Tidak Diketahui";

// ================= VARIABEL KOMUNIKASI ANTAR CORE =================
// Flag untuk menandakan ada data mentah baru yang siap diproses
volatile bool newDataReady = false;

// Buffer data untuk dilempar dari Core 1 ke Core 0
volatile unsigned int sharedSignalMax[numSensors];
volatile unsigned int sharedSignalMin[numSensors];
volatile unsigned long sharedTimeArrivals[numSensors];

// Task Handle untuk Core 0
TaskHandle_t TaskCore0Handle;

// =================================================

void setup() {
  Serial.begin(115200);
  
  // Inisialisasi WiFi dilakukan di setup (biasanya berjalan di Core 1 pada saat booting)
  setup_wifi();
  client.setServer(mqtt_server, 1883);

  // Membuat Task baru yang dipaku (pinned) secara khusus di CORE 0
  xTaskCreatePinnedToCore(
    core0Task,          // Nama fungsi task
    "MQTT_Math_Task",   // Nama task (untuk debugging)
    10000,              // Ukuran stack (byte) - diperbesar karena ada MQTT & String
    NULL,               // Parameter task
    1,                  // Prioritas task (1 adalah normal)
    &TaskCore0Handle,   // Handle task
    0                   // Dipaku di Core 0
  );

  // Header untuk Serial Monitor (Opsional, agar tahu urutan kolomnya)
  Serial.println("Depan\tKiri\tBelakang\tKanan\tAVG\tStatus\tSudut\tArah");
}

// ================= CORE 1: PENGAMBILAN DATA (TELINGA) =================
// loop() bawaan Arduino secara default selalu berjalan di Core 1
void loop() {
  unsigned int localSignalMax[numSensors] = {0, 0, 0, 0};
  unsigned int localSignalMin[numSensors] = {4095, 4095, 4095, 4095};
  unsigned long localTimeArrivals[numSensors] = {0, 0, 0, 0};

  unsigned long startMillis = millis();

  // SAMPLING 50ms NON-STOP
  while (millis() - startMillis < sampleWindow) {
    for (int i = 0; i < numSensors; i++) {
      int sample = analogRead(sensorPins[i]);

      if (sample > localSignalMax[i]) {
        localSignalMax[i] = sample;
        localTimeArrivals[i] = micros(); 
      }
      if (sample < localSignalMin[i]) {
        localSignalMin[i] = sample;
      }
    }
  }

  // Jika Core 0 sudah selesai memproses data sebelumnya, kirim data terbaru ini
  if (!newDataReady) {
    for (int i = 0; i < numSensors; i++) {
      sharedSignalMax[i] = localSignalMax[i];
      sharedSignalMin[i] = localSignalMin[i];
      sharedTimeArrivals[i] = localTimeArrivals[i];
    }
    newDataReady = true; // Beri sinyal ke Core 0 bahwa data siap
  }
}

// ================= CORE 0: PEMROSESAN & JARINGAN (OTAK) =================
void core0Task(void * pvParameters) {
  unsigned long lastMQTTUpdate = 0;

  for(;;) {
    // Pastikan MQTT tetap terkoneksi
    if (!client.connected()) reconnectMQTT();
    client.loop();

    // Jika Core 1 mengirimkan data baru
    if (newDataReady) {
      
      float dbValues[numSensors];
      float totalDB = 0;

      // 1. HITUNG dB
      // Melakukan perulangan untuk memproses data dari keempat sensor (Depan, Kiri, Belakang, Kanan)
      for (int i = 0; i < numSensors; i++) {
        // Menghitung Amplitudo (Peak-to-Peak)
        // Selisih antara lonjakan gelombang tertinggi dan terendah yang ditangkap oleh Core 1 selama 50ms.
        int peakToPeak = sharedSignalMax[i] - sharedSignalMin[i];
        // Konversi ADC ke Tegangan (Volt)
        // ESP32 memiliki resolusi ADC 12-bit (0 - 4095) dan beroperasi pada tegangan referensi 3.3 Volt.
        float volts = (peakToPeak * 3.3) / 4095.0;
        // Mengubah tegangan Peak-to-Peak menjadi tegangan RMS (Root Mean Square)
        // Rumus Vrms dari Vpp adalah Vpp / (2 * akar 2), yang nilainya sama dengan dikali 0.3535.
        // Vrms merepresentasikan energi efektif / daya sebenarnya dari gelombang suara tersebut.
        float vRms = volts * 0.3535;

        // Inisialisasi nilai awal desibel mentah
        float rawDB = 0;
        // Syarat pencegahan Error (NaN / Infinity)
        // Nilai logaritma dari 0 atau negatif tidak terdefinisi. Jadi, perhitungan hanya dilakukan jika ada suara (vRms > 0.001 Volt).
        if (vRms > 0.001) {
          // Rumus standar akustik untuk menghitung Sound Pressure Level (SPL) dalam desibel.
          // 20 * log10(Vrms) ditambah dengan konstanta offset kalibrasi dasar sensor.
          rawDB = 20.0 * log10(vRms) + calibrationOffset;
        }

        // Tahap Kalibrasi SLM (Sound Level Meter)
        // Memasukkan desibel mentah ke fungsi interpolasi agar nilainya akurat dan identik dengan alat ukur SLM standar.
        dbValues[i] = interpolateDB(rawDB);
        // Membatasi nilai bawah agar tidak muncul angka minus saat ruangan benar-benar sunyi senyap.
        if (dbValues[i] < 0) dbValues[i] = 0;
        
        // Menjumlahkan nilai dB dari keempat sensor untuk dicari rata-ratanya nanti.
        totalDB += dbValues[i];
      }

      // Menghitung Rata-rata tingkat kebisingan dari keempat sisi mikrofon.
      float avgDB = totalDB / numSensors;
      // Menentukan Status Ruangan menggunakan (Ternary Operator).
      // Jika rata-rata dB menyentuh atau melewati batas (misal 55 dB), status = "Bising". Jika di bawahnya = "Normal".
      String status = (avgDB >= dbThreshold) ? "Bising" : "Normal";

      // 2. PENENTUAN ARAH (ONE-SHOT TDOA)
      
      // Inisialisasi nilai awal. Sudut -1 menandakan belum ada arah yang valid.
      float finalAzimuth = -1; 
      String arah = "Tidak Diketahui";

      // Pemicu Utama: TDOA hanya diproses JIKA rata-rata suara melewati ambang batas (contoh: >= 55 dB)
      if (avgDB >= dbThreshold) {
        // Cek Status Kunci: Jika suara baru saja muncul (belum terkunci), lakukan perhitungan.
        if (!isSoundLocked) {
          // Pastikan keempat sensor benar-benar menangkap waktu puncak (waktu > 0)
          if (sharedTimeArrivals[0] > 0 && sharedTimeArrivals[1] > 0 && 
              sharedTimeArrivals[2] > 0 && sharedTimeArrivals[3] > 0) {

            // Menghitung selisih waktu (Delta) antar sensor yang berlawanan.
            // Sumbu Y (Depan-Belakang). Jika waktu Belakang lebih besar (suara sampai di Depan duluan), dY bernilai positif.
            long dY = (long)(sharedTimeArrivals[2] - sharedTimeArrivals[0]);
            // Sumbu X (Kiri-Kanan). Jika waktu Kiri lebih besar (suara sampai di Kanan duluan), dX bernilai positif.
            long dX = (long)(sharedTimeArrivals[1] - sharedTimeArrivals[3]);

            // Menggunakan fungsi atan2 untuk mencari sudut (dalam Radian), lalu dikonversi ke Derajat (* 180 / PI)
            // Format atan2(X, Y) digunakan agar 0 derajat berada tepat di Depan.
            finalAzimuth = atan2((float)dX, (float)dY) * 180.0 / PI;
            
            // Normalisasi sudut: Jika hasilnya negatif (misal -90 derajat), ubah menjadi format 0-360 derajat (jadi 270 derajat).
            if (finalAzimuth < 0) finalAzimuth += 360;
            
            // Mengubah angka derajat menjadi teks (misal: "Depan-Kanan")
            arah = getDirectionLabel(finalAzimuth);

            // TAHAP MENGUNCI (ONE-SHOT):
            // Simpan hasil perhitungan arah ini ke variabel memori.
            lockedAzimuth = finalAzimuth;
            lockedArah = arah;
            // Aktifkan kunci agar jika looping berikutnya suara masih di atas 55 dB, sistem tidak perlu menghitung ulang.
            isSoundLocked = true; 
          }
        } else {
          // JIKA SISTEM TERKUNCI:
          // Artinya suara bising masih terus berlangsung sejak getaran pertama.
          // Gunakan data arah yang sudah disimpan sebelumnya tanpa melakukan perhitungan atan2 lagi.
          finalAzimuth = lockedAzimuth;
          arah = lockedArah;
          status = "Bising";
        }
      } else {
        // JIKA SUARA MEREDA (Di bawah ambang batas):
        // Lepaskan kunci dan reset semua memori arah agar sistem siap mendeteksi suara baru.
        isSoundLocked = false;
        lockedAzimuth = -1;
        lockedArah = "Tidak Diketahui";
      }

      // Beri izin Core 1 untuk mengirim data baru lagi
      newDataReady = false; 

      // 3. KIRIM KE MQTT & SERIAL (Dibatasi 1 detik agar tidak spamming server)
      if (millis() - lastMQTTUpdate > 1000) {
        
        // Print Serial Format Tabel
        Serial.printf("%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%s\t", 
                      dbValues[0], dbValues[1], dbValues[2], dbValues[3], avgDB, status.c_str());
        
        if (finalAzimuth != -1) {
          Serial.printf("%.0f\t%s\n", finalAzimuth, arah.c_str());
        } else {
          Serial.printf("-\t-\n"); // Jika di bawah 55dB, sudut dan arah dicetak sebagai strip (-)
        }

        // Kirim MQTT
        if (client.connected()) {
          char payload[256];
          snprintf(payload, sizeof(payload),
            "{\"depan\":%.1f,\"kiri\":%.1f,\"belakang\":%.1f,\"kanan\":%.1f,\"avg\":%.1f,\"status\":\"%s\",\"azimuth\":%.0f,\"arah\":\"%s\"}",
            dbValues[0], dbValues[1], dbValues[2], dbValues[3],      
            avgDB, status.c_str(), finalAzimuth, arah.c_str()
          );
          client.publish("v1/devices/me/telemetry", payload);
        }

        lastMQTTUpdate = millis();
      }
    }

    // Wajib ada untuk mencegah Watchdog Timer Reset pada ESP32
    vTaskDelay(5 / portTICK_PERIOD_MS); 
  }
}

// ================= FUNGSI BANTUAN =================

float interpolateDB(float rawDB) {
  if (rawDB <= rawSensorPoints[0]) return trueSLMPoints[0] - (rawSensorPoints[0] - rawDB);
  if (rawDB >= rawSensorPoints[numPoints - 1]) return trueSLMPoints[numPoints - 1] + (rawDB - rawSensorPoints[numPoints - 1]);

  for (int i = 0; i < numPoints - 1; i++) {
    if (rawDB >= rawSensorPoints[i] && rawDB <= rawSensorPoints[i+1]) {
      return trueSLMPoints[i] + ((rawDB - rawSensorPoints[i]) * (trueSLMPoints[i+1] - trueSLMPoints[i]) / (rawSensorPoints[i+1] - rawSensorPoints[i]));
    }
  }
  return rawDB;
}

String getDirectionLabel(float deg) {
  if (deg >= 337 || deg < 22) return "Depan";
  if (deg < 67) return "Depan-Kanan";
  if (deg < 112) return "Kanan";
  if (deg < 157) return "Belakang-Kanan";
  if (deg < 202) return "Belakang";
  if (deg < 247) return "Belakang-Kiri";
  if (deg < 292) return "Kiri";
  return "Depan-Kiri";
}

void setup_wifi() {
  Serial.print("Connecting WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected");
}

void reconnectMQTT() {
  if (!client.connected()) {
    client.connect("ESP32Client", TOKEN, NULL);
  }
}
