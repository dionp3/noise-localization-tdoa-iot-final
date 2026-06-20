#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "NamaWIFI?";
const char* password = "PasswordWIFI?";
const char* mqtt_server = "ServerMQTTAddress?"; 
const char* TOKEN = "TokenDeviceThingsboard?"; 

WiFiClient espClient;
PubSubClient client(espClient);

const int numSensors = 4;
const int pins[numSensors] = {34, 35, 32, 33}; 

const int SENSITIVITY_OFFSET = 50;
const unsigned long TIMEOUT_US = 6000;
const long CENTER_TOLERANCE_US = 300;
int THRESHOLD = 2048;

void setup_wifi() {
  Serial.print("Connecting WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { 
    delay(500); 
    Serial.print("."); 
  }
  Serial.printf("\nWiFi Connected\nIP: %s\n", WiFi.localIP().toString().c_str());
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Connecting MQTT...");
    if (client.connect("ESP32Client", TOKEN, NULL)) {
      Serial.println(" Connected");
    } else { 
      Serial.printf(" Failed, rc=%d Retry...\n", client.state()); 
      delay(2000); 
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  analogReadResolution(12);
  setup_wifi();
  client.setServer(mqtt_server, 1883);

  Serial.print("Mengkalibrasi ruangan...");
  long avgCenter = 0;
  for (int i = 0; i < 200; i++) {
    avgCenter += analogRead(pins[0]);
    if (i % 20 == 0) Serial.print(".");
    delay(2);
  }
  THRESHOLD = (avgCenter / 200) + SENSITIVITY_OFFSET;
  Serial.printf("\nSukses. Threshold: %d\n\nT_Depan(us)\tT_Kiri(us)\tT_Belakang(us)\tT_Kanan(us)\tAzimuth(°)\tArah\n", THRESHOLD);
}

void loop() {
  if (!client.connected()) reconnectMQTT();
  client.loop();

  unsigned long t_depan = 0;
  unsigned long t_kiri = 0;
  unsigned long t_belakang = 0;
  unsigned long t_kanan = 0;
  int triggered_count = 0;
  unsigned long start_time = 0;

  while (triggered_count < 4) {
    if (t_depan == 0 && analogRead(pins[0]) > THRESHOLD) {
      t_depan = micros();
      triggered_count = triggered_count + 1;
      if (triggered_count == 1) start_time = t_depan;
    }
    
    if (t_kiri == 0 && analogRead(pins[1]) > THRESHOLD) {
      t_kiri = micros();
      triggered_count = triggered_count + 1;
      if (triggered_count == 1) start_time = t_kiri;
    }
    
    if (t_belakang == 0 && analogRead(pins[2]) > THRESHOLD) {
      t_belakang = micros();
      triggered_count = triggered_count + 1;
      if (triggered_count == 1) start_time = t_belakang;
    }
    
    if (t_kanan == 0 && analogRead(pins[3]) > THRESHOLD) {
      t_kanan = micros();
      triggered_count = triggered_count + 1;
      if (triggered_count == 1) start_time = t_kanan;
    }

    if (triggered_count > 0 && (micros() - start_time > TIMEOUT_US)) {
      break;
    }
  }

  if (triggered_count < 4) return;

  long dY = (long)(t_belakang - t_depan);
  long dX = (long)(t_kiri - t_kanan); 
  String arah_teks = "";
  float azimuth = 0;

  if (labs(dY) <= CENTER_TOLERANCE_US && labs(dX) <= CENTER_TOLERANCE_US) {
    arah_teks = "Tengah";
    azimuth = -1.0;
  } else {
    azimuth = atan2((float)dX, (float)dY) * 180.0 / PI;
    if (azimuth < 0) {
      azimuth = azimuth + 360.0;
    }
    
    if (azimuth >= 337 || azimuth <= 21) {
      arah_teks = "Depan";
    } else if (azimuth >= 22 && azimuth <= 66) {
      arah_teks = "Depan-Kanan";
    } else if (azimuth >= 67 && azimuth <= 111) {
      arah_teks = "Kanan";
    } else if (azimuth >= 112 && azimuth <= 156) {
      arah_teks = "Belakang-Kanan";
    } else if (azimuth >= 157 && azimuth <= 201) {
      arah_teks = "Belakang";
    } else if (azimuth >= 202 && azimuth <= 246) {
      arah_teks = "Belakang-Kiri";
    } else if (azimuth >= 247 && azimuth <= 291) {
      arah_teks = "Kiri";
    } else if (azimuth >= 292 && azimuth <= 336) {
      arah_teks = "Depan-Kiri";
    }
  }

  Serial.printf("%lu\t%lu\t%lu\t%lu\t", t_depan, t_kiri, t_belakang, t_kanan);
  if (azimuth != -1.0) {
    Serial.printf("%.1f\t%s\n", azimuth, arah_teks.c_str());
  } else {
    Serial.printf("-\t%s\n", arah_teks.c_str());
  }

  if (client.connected()) {
    char payload[256];
    snprintf(payload, sizeof(payload),
             "{\"t_depan\":%lu,\"t_kiri\":%lu,\"t_belakang\":%lu,\"t_kanan\":%lu,\"dx\":%ld,\"dy\":%ld,\"azimuth\":%.0f,\"arah\":\"%s\"}",
             t_depan, t_kiri, t_belakang, t_kanan, dX, dY, azimuth, arah_teks.c_str());
    client.publish("v1/devices/me/telemetry", payload);
  }
  
  delay(800);
}
