#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ================= WIFI & MQTT =================
const char* ssid = "NamaWIFI?";
const char* password = "PasswordWIFI?";
const char* mqtt_server = "ServerMQTTAddress?"; 
const char* TOKEN = "TokenDeviceThingsboard?"; 

WiFiClient espClient;
PubSubClient client(espClient);

// ================= PIN MIKROFON =================
const int MIC_DEPAN_PIN    = 34;
const int MIC_KIRI_PIN     = 35;
const int MIC_BELAKANG_PIN = 32;
const int MIC_KANAN_PIN    = 33;

// ================= PARAMETER =================
const int SENSITIVITY_OFFSET = 50;
const unsigned long TIMEOUT_US = 6000;
const long CENTER_TOLERANCE_US = 300;

int THRESHOLD = 2048;

// ================= WIFI =================
void setup_wifi() {
  Serial.print("Connecting WiFi...");
  
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi Connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

// ================= MQTT =================
void reconnectMQTT() {
  while (!client.connected()) {

    Serial.print("Connecting MQTT...");

    if (client.connect("ESP32Client", TOKEN, NULL)) {
      Serial.println(" Connected");
    } else {
      Serial.print(" Failed, rc=");
      Serial.print(client.state());
      Serial.println(" Retry...");
      delay(2000);
    }
  }
}

// ================= SETUP =================
void setup() {

  Serial.begin(115200);
  delay(1000);

  while (!Serial);

  analogReadResolution(12);

  setup_wifi();
  client.setServer(mqtt_server, 1883);

  Serial.println("Mengkalibrasi ruangan...");

  long avgCenter = 0;

  for (int i = 0; i < 200; i++) {

    avgCenter += analogRead(MIC_DEPAN_PIN);

    if (i % 20 == 0) {
      Serial.print(".");
    }

    delay(2);
  }

  Serial.println();

  THRESHOLD = (avgCenter / 200) + SENSITIVITY_OFFSET;

  Serial.printf("Kalibrasi Sukses. Threshold: %d\n\n", THRESHOLD);

  Serial.println("T_Depan(us)\tT_Kiri(us)\tT_Belakang(us)\tT_Kanan(us)\tAzimuth(°)\tArah");
}

// ================= LOOP =================
void loop() {

  if (!client.connected()) {
    reconnectMQTT();
  }

  client.loop();

  unsigned long t_depan = 0;
  unsigned long t_kiri = 0;
  unsigned long t_belakang = 0;
  unsigned long t_kanan = 0;

  int triggered_count = 0;
  unsigned long start_time = 0;

  while (triggered_count < 4) {

    if (t_depan == 0 && analogRead(MIC_DEPAN_PIN) > THRESHOLD) {

      t_depan = micros();

      if (++triggered_count == 1) {
        start_time = t_depan;
      }
    }

    if (t_kiri == 0 && analogRead(MIC_KIRI_PIN) > THRESHOLD) {

      t_kiri = micros();

      if (++triggered_count == 1) {
        start_time = t_kiri;
      }
    }

    if (t_belakang == 0 && analogRead(MIC_BELAKANG_PIN) > THRESHOLD) {

      t_belakang = micros();

      if (++triggered_count == 1) {
        start_time = t_belakang;
      }
    }

    if (t_kanan == 0 && analogRead(MIC_KANAN_PIN) > THRESHOLD) {

      t_kanan = micros();

      if (++triggered_count == 1) {
        start_time = t_kanan;
      }
    }

    if (triggered_count > 0 &&
        (micros() - start_time > TIMEOUT_US)) {
      break;
    }
  }

  if (triggered_count < 4) {
    return;
  }

  long dY = (long)(t_belakang - t_depan);
  long dX = (long)(t_kiri - t_kanan);

  String arah_teks = "";
  float azimuth = 0;

  if (labs(dY) <= CENTER_TOLERANCE_US &&
      labs(dX) <= CENTER_TOLERANCE_US) {

    arah_teks = "Tengah";
    azimuth = -1.0;
  }
  else {

    azimuth = atan2((float)dX, (float)dY) * 180.0 / PI;

    if (azimuth < 0) {
      azimuth += 360.0;
    }

    if (azimuth >= 337.5 || azimuth < 22.5)
      arah_teks = "Depan";

    else if (azimuth >= 22.5 && azimuth < 67.5)
      arah_teks = "Depan-Kanan";

    else if (azimuth >= 67.5 && azimuth < 112.5)
      arah_teks = "Kanan";

    else if (azimuth >= 112.5 && azimuth < 157.5)
      arah_teks = "Belakang-Kanan";

    else if (azimuth >= 157.5 && azimuth < 202.5)
      arah_teks = "Belakang";

    else if (azimuth >= 202.5 && azimuth < 247.5)
      arah_teks = "Belakang-Kiri";

    else if (azimuth >= 247.5 && azimuth < 292.5)
      arah_teks = "Kiri";

    else
      arah_teks = "Depan-Kiri";
  }

  Serial.printf("%lu\t%lu\t%lu\t%lu\t",
                t_depan,
                t_kiri,
                t_belakang,
                t_kanan);

  if (azimuth != -1.0) {
    Serial.printf("%.1f\t%s\n",
                  azimuth,
                  arah_teks.c_str());
  }
  else {
    Serial.printf("-\t%s\n",
                  arah_teks.c_str());
  }

  // ================= MQTT TELEMETRY =================
  if (client.connected()) {

    char payload[256];

    snprintf(payload, sizeof(payload),
             "{\"t_depan\":%lu,"
             "\"t_kiri\":%lu,"
             "\"t_belakang\":%lu,"
             "\"t_kanan\":%lu,"
             "\"dx\":%ld,"
             "\"dy\":%ld,"
             "\"azimuth\":%.0f,"
             "\"arah\":\"%s\"}",
             t_depan,
             t_kiri,
             t_belakang,
             t_kanan,
             dX,
             dY,
             azimuth,
             arah_teks.c_str());

    client.publish("v1/devices/me/telemetry", payload);
  }

  delay(800);
}
