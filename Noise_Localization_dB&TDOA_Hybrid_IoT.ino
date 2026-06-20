#include <WiFi.h>
#include <math.h>
#include <PubSubClient.h>

const char* ssid = "NamaWIFI?";
const char* password = "PasswordWIFI?";
const char* mqtt_server = "ServerMQTTAddress?"; 
const char* TOKEN = "TokenDeviceThingsboard?"; 

WiFiClient espClient;
PubSubClient client(espClient);

const int numSensors = 4;
const int sensorPins[numSensors] = {34, 35, 32, 33};

const int sampleWindow = 50;
const float dbThreshold = 55.0; 
const float calibrationOffset = 76.0; 

const float rawSensorPoints[] = {43.60, 45.12, 46.86, 49.61, 55.79, 64.99}; 
const float trueSLMPoints[]   = {45.0, 51.0, 55.5, 60.5, 71.0, 80.0};
const int numPoints = 6;

const float MAX_DB_DIFF = 12.0;       
const float MAX_TDOA_DIFF_US = 200.0; 

volatile bool newDataReady = false;
volatile unsigned int sharedSignalMax[numSensors];
volatile unsigned int sharedSignalMin[numSensors];
volatile unsigned long sharedTimeArrivals[numSensors];

TaskHandle_t TaskCore0Handle;

void core0Task(void * pvParameters);
float interpolateDB(float rawDB);
String getDirectionLabel(float deg);
void setup_wifi();
void reconnectMQTT();
void hitungDesibelPerSensor(float *dbValues, float &totalDB);
void hitungTDOA(float &tdoa_dX, float &tdoa_dY, float &sudutTDOA, String &arahTDOA);
void hitungHybridFusion(float *dbValues, float &finalAzimuth, String &arah);

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  xTaskCreatePinnedToCore(core0Task, "MQTT_Math_Task", 10000, NULL, 1, &TaskCore0Handle, 0);
  Serial.println("Depan\tKiri\tBelakang\tKanan\tAVG\tStatus\tT_Depan(us)\tT_Kiri(us)\tT_Belakang(us)\tT_Kanan(us)\tSudut\tArah");
}

void loop() {
  unsigned int localSignalMax[numSensors] = {0, 0, 0, 0};
  unsigned int localSignalMin[numSensors] = {4095, 4095, 4095, 4095};
  unsigned long localTimeArrivals[numSensors] = {0, 0, 0, 0};
  unsigned long startMillis = millis();
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
  if (!newDataReady) {
    for (int i = 0; i < numSensors; i++) {
      sharedSignalMax[i] = localSignalMax[i];
      sharedSignalMin[i] = localSignalMin[i];
      sharedTimeArrivals[i] = localTimeArrivals[i];
    }
    newDataReady = true; 
  }
}

void core0Task(void * pvParameters) {
  unsigned long lastMQTTUpdate = 0;
  for(;;) {
    if (!client.connected()) reconnectMQTT();
    client.loop();
    if (newDataReady) {
      float dbValues[numSensors];
      float totalDB = 0;
      hitungDesibelPerSensor(dbValues, totalDB);
      float avgDB = totalDB / numSensors;
      String status = (avgDB >= dbThreshold) ? "Bising" : "Normal";
      float finalAzimuth = -1;
      String arah = "Tidak Diketahui";
      if (avgDB >= dbThreshold) {
        if (sharedTimeArrivals[0] > 0 && sharedTimeArrivals[1] > 0 &&
            sharedTimeArrivals[2] > 0 && sharedTimeArrivals[3] > 0) {
          hitungHybridFusion(dbValues, finalAzimuth, arah); 
        }
      } 
      newDataReady = false; 
      if (millis() - lastMQTTUpdate > 1000) {
        Serial.printf("%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%s\t", dbValues[0], dbValues[1], dbValues[2], dbValues[3], avgDB, status.c_str());
        
        if (avgDB >= dbThreshold) {
          Serial.printf("%lu\t%lu\t%lu\t%lu\t", sharedTimeArrivals[0], sharedTimeArrivals[1], sharedTimeArrivals[2], sharedTimeArrivals[3]);
          Serial.printf("%.0f\t%s\n", finalAzimuth, arah.c_str());
        } else {
          Serial.printf("-\t-\t-\t-\t-\t-\n");
        }

        if (client.connected()) {
          char payload[256];
          snprintf(payload, sizeof(payload),
            "{\"depan\":%.1f,\"kiri\":%.1f,\"belakang\":%.1f,\"kanan\":%.1f,\"avg\":%.1f,\"status\":\"%s\",\"azimuth\":%.0f,\"arah\":\"%s\"}",
            dbValues[0], dbValues[1], dbValues[2], dbValues[3], avgDB, status.c_str(), finalAzimuth, arah.c_str());
          client.publish("v1/devices/me/telemetry", payload);
        }
        lastMQTTUpdate = millis();
      }
    }
    vTaskDelay(5 / portTICK_PERIOD_MS); 
  }
}

void hitungDesibelPerSensor(float *dbValues, float &totalDB) {
  for (int i = 0; i < numSensors; i++) {
    int peakToPeak = sharedSignalMax[i] - sharedSignalMin[i];
    float volts = (peakToPeak * 3.3) / 4095.0;
    float vRms = volts * 0.3535; 
    float rawDB = 0;
    if (vRms > 0.001) {
      rawDB = 20.0 * log10(vRms) + calibrationOffset;
    }
    dbValues[i] = interpolateDB(rawDB);
    if (dbValues[i] < 0) dbValues[i] = 0;
    totalDB += dbValues[i];
  }
}

void hitungTDOA(float &tdoa_dX, float &tdoa_dY, float &sudutTDOA, String &arahTDOA) {
  tdoa_dY = (long)(sharedTimeArrivals[2] - sharedTimeArrivals[0]); 
  tdoa_dX = (long)(sharedTimeArrivals[1] - sharedTimeArrivals[3]); 
  sudutTDOA = atan2(tdoa_dX, tdoa_dY) * 180.0 / PI;
  if (sudutTDOA < 0) sudutTDOA += 360;
  arahTDOA = getDirectionLabel(sudutTDOA);
}

void hitungHybridFusion(float *dbValues, float &finalAzimuth, String &arah) {
  float db_dY = dbValues[0] - dbValues[2]; 
  float db_dX = dbValues[3] - dbValues[1]; 
  float norm_db_dY = constrain(db_dY / MAX_DB_DIFF, -1.0, 1.0);
  float norm_db_dX = constrain(db_dX / MAX_DB_DIFF, -1.0, 1.0);
  float tdoa_dX = 0, tdoa_dY = 0, sudutDummy = 0;
  String arahDummy = "";
  hitungTDOA(tdoa_dX, tdoa_dY, sudutDummy, arahDummy);
  float norm_tdoa_dY = constrain(tdoa_dY / MAX_TDOA_DIFF_US, -1.0, 1.0);
  float norm_tdoa_dX = constrain(tdoa_dX / MAX_TDOA_DIFF_US, -1.0, 1.0);
  float hybrid_dY = (0.7 * norm_db_dY) + (0.3 * norm_tdoa_dY);
  float hybrid_dX = (0.7 * norm_db_dX) + (0.3 * norm_tdoa_dX);
  finalAzimuth = atan2(hybrid_dX, hybrid_dY) * 180.0 / PI;
  if (finalAzimuth < 0) finalAzimuth += 360;
  arah = getDirectionLabel(finalAzimuth);
}

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
