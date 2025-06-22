#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <math.h>

// --- Pengaturan untuk SIM800L dan MQTT ---
#define TINY_GSM_MODEM_SIM800 
// #define TINY_GSM_DEBUG Serial // Uncomment untuk debug AT command ke Serial Monitor utama

#define SIM800_TX_PIN 10 
#define SIM800_RX_PIN 11 
//#define SIM800_RST_PIN 5 // Opsional

HardwareSerial simSerial(2); 

#include <TinyGsmClient.h>
#include <PubSubClient.h>

const char apn[]  = "internet"; // GANTI SESUAI PROVIDER SIM ANDA
const char gprsUser[] = ""; 
const char gprsPass[] = ""; 

const char* mqtt_broker = "broker.hivemq.com";
const int   mqtt_port = 1883;
const char* mqtt_topic_publish = "sim800l/data";
char mqtt_client_id[30];


TinyGsm modem(simSerial);
TinyGsmClient client(modem); 
PubSubClient mqtt(client);   
// --- Akhir Pengaturan SIM800L dan MQTT ---

// === Kredensial WiFi === (TIDAK DIGUNAKAN)

// === Definisi Pin & Konstanta Umum ===
#define OLED_SDA 8
#define OLED_SCL 9
#define MPU_SDA 13
#define MPU_SCL 6
#define MPU_ADDR 0x68
#define GPS_RX 16 
#define GPS_TX 17 
const int BUTTON_PIN = 2; 

#define G_ACCEL 9.80665
const float ACCEL_RAW_TO_MSS = G_ACCEL / 16384.0;
const float GYRO_RAW_TO_DPS = 1.0 / 131.0;
#define RAD_TO_DEG 57.295779513
#define DEG_TO_RAD 0.01745329251
const float YAW_FILTER_ALPHA = 0.98;
const float MIN_SPEED_FOR_COG = 0.5;
const unsigned long LOGGING_INTERVAL_MS = 5000; 
const unsigned long DEBOUNCE_DELAY_MS = 50;

// === Objek Sensor ===
Adafruit_SSD1306 display(128, 32, &Wire, -1);
TwoWire WireMPU = TwoWire(1);
MPU6050 mpu(MPU_ADDR, &WireMPU);
SFE_UBLOX_GNSS myGNSS; 

// === Variabel Global Sensor & Kalibrasi ===
int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
const float ax_offset_mss = 0.197742; 
const float ay_offset_mss = 0.207984; 
const float az_offset_mss = 9.437900; 
const float gx_offset_dps = -0.012285;
const float gy_offset_dps = 0.022122; 
const float gz_offset_dps = 0.010568; 
float accX_mss, accY_mss, accZ_mss;
float gyroX_dps, gyroY_dps, gyroZ_dps;
float gyroX_rps, gyroY_rps, gyroZ_rps;

// === Variabel Global Waktu & Filter Roll/Pitch ===
unsigned long lastUpdateTime = 0;
float dt = 0; 
float angle_roll = 0.0f; float bias_roll = 0.0f; float P_roll[2][2] = {{1,0},{0,1}};
float angle_pitch = 0.0f; float bias_pitch = 0.0f; float P_pitch[2][2] = {{1,0},{0,1}};
float roll_acc_deg = 0.0f; float pitch_acc_deg = 0.0f;

// === Variabel Global Data GPS ===
static float lat_deg = 0.0; static float lon_deg = 0.0; static float alt_m = 0.0;
static float spd_mps = 0.0; static float velN_mps = 0.0; static float velE_mps = 0.0; static float velD_mps = 0.0;
static int fix_type = 0; 
static int sats = 0;
static char wibTimeSerial[40] = "Waiting..."; 
static int oled_year=0, oled_month=0, oled_day=0;
static int oled_hour=0, oled_minute=0, oled_second=0;

// === Variabel Global Estimasi Yaw ===
float estYawRad = 0.0; float gpsCOG_rad = 0.0; bool validGpsCOG = false;

// === Variabel Global Logging & Kontrol Limit Switch ===
unsigned long lastLogTimeMQTT = 0; 
bool sendingDataActive = false; 
unsigned long lastButtonPressTime = 0; 
bool lastButtonState = HIGH; 

// === Variabel Global Filter Kecepatan ===
float kf_estimated_speed_mps = 0.0f;
float kf_speed_accel_bias_mps2 = 0.0f;
float kf_speed_P[2][2] = {{1,0},{0,1}};
const float KF_SPEED_Q_SPEED = 0.01f; 
const float KF_SPEED_Q_BIAS = 0.003f;
const float KF_SPEED_R_MEASURE = 0.5f;

unsigned long lastMqttConnectAttempt = 0;
const unsigned long MQTT_RECONNECT_INTERVAL = 5000; 


// -------------------------------------------------------
// Fungsi Helper untuk mendapatkan Tipe Fix GPS sebagai String
// -------------------------------------------------------
String getFixTypeString(int fixVal) {
    switch (fixVal) {
        case 0: return "NO";  
        case 1: return "DR";  
        case 2: return "2D";
        case 3: return "3D";
        case 4: return "GNSS+DR";
        case 5: return "Time";
        default: return String(fixVal); 
    }
}

// -------------------------------------------------------
// Fungsi Kalman Filter Generik
// -------------------------------------------------------
void kalmanUpdateGeneric(float newMeasurement, float newRateMeasurement, float dt_filter,
                         float &estimate, float &bias, float P[2][2],
                         float Q_estimate_process, float Q_bias_process, float R_measurement) {
    if (dt_filter <= 1e-6) return; 

    estimate += dt_filter * (newRateMeasurement - bias);
    P[0][0] += dt_filter * (dt_filter * P[1][1] - P[0][1] - P[1][0] + Q_estimate_process);
    P[0][1] -= dt_filter * P[1][1];
    P[1][0] -= dt_filter * P[1][1];
    P[1][1] += Q_bias_process * dt_filter;
    float S = P[0][0] + R_measurement; 
    if (abs(S) < 1e-9) return; 

    float K[2]; 
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;
    float y = newMeasurement - estimate; 
    estimate += K[0] * y;
    bias += K[1] * y;
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];
    P[0][0] -= K[0] * P00_temp; P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp; P[1][1] -= K[1] * P01_temp;
}

// -------------------------------------------------------
// Fungsi Normalisasi Sudut
// -------------------------------------------------------
float normalizeAngle(float angleRad) {
    while (angleRad > M_PI) angleRad -= 2.0 * M_PI;
    while (angleRad <= -M_PI) angleRad += 2.0 * M_PI;
    return angleRad;
}
// -------------------------------------------------------
// Fungsi Membuat JSON Data Periodik
// -------------------------------------------------------
String createPeriodicJson(float gps_speed_kmh, float kalman_filtered_speed_kmh, float actual_dt_ms) {
    StaticJsonDocument <2048> jsonDoc; 
    jsonDoc["lat"] = lat_deg;
    jsonDoc["lon"] = lon_deg;
    jsonDoc["alt"] = alt_m;
    jsonDoc["fix"] = fix_type; 
    jsonDoc["sats"] = sats;
    jsonDoc["accX"] = accX_mss;
    jsonDoc["accY"] = accY_mss;
    jsonDoc["accZ"] = accZ_mss;
    jsonDoc["gyroX"] = gyroX_rps;
    jsonDoc["gyroY"] = gyroY_rps;
    jsonDoc["gyroZ"] = gyroZ_rps;
    jsonDoc["roll"] = angle_roll;
    jsonDoc["pitch"] = angle_pitch;
    jsonDoc["yaw"] = estYawRad * RAD_TO_DEG;
    jsonDoc["gpsSpdKmh"] = gps_speed_kmh;
    jsonDoc["kfSpdKmh"] = kalman_filtered_speed_kmh;
    jsonDoc["cogOk"] = validGpsCOG;
    jsonDoc["dtMs"] = actual_dt_ms; 

    String outputJson;
    serializeJson(jsonDoc, outputJson);
    if (outputJson.length() == 0 && jsonDoc.capacity() > 0) {
        Serial.println("ERROR: Gagal serialisasi JSON Periodik! Buffer terlalu kecil?");
    }
    return outputJson;
}

// Fungsi callback untuk pesan MQTT yang masuk
void mqttCallback(char* topic, byte* payload, unsigned int len) {
    Serial.print("Pesan diterima [");
    Serial.print(topic);
    Serial.print("]: ");
    for (unsigned int i = 0; i < len; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();
}

// Fungsi untuk koneksi ke MQTT Broker
void connectMQTT() {
    if (mqtt.connected()) {
        return;
    }
    Serial.print("Menyambungkan ke MQTT broker ");
    Serial.print(mqtt_broker);
    Serial.print("...");

    String imei = modem.getIMEI();
    if (imei.length() > 0) {
        snprintf(mqtt_client_id, sizeof(mqtt_client_id), "esp32_sim800_%s", imei.c_str());
    } else {
        snprintf(mqtt_client_id, sizeof(mqtt_client_id), "esp32_sim800_%ld", random(0xffff));
    }
    Serial.print(" dengan Client ID: ");
    Serial.println(mqtt_client_id);

    if (mqtt.connect(mqtt_client_id)) { 
        Serial.println(" tersambung!");
    } else {
        Serial.print(" gagal, rc=");
        Serial.print(mqtt.state());
        Serial.println(" coba lagi dalam beberapa detik");
    }
}


// =======================================================
//             SETUP
// =======================================================
void setup() {
    Serial.begin(115200);
    Serial.println("\nMemulai Setup Sistem dengan SIM800L MQTT...");

    // --- Inisialisasi Limit Switch ---
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    bool initialSwitchReading = digitalRead(BUTTON_PIN);
    lastButtonState = initialSwitchReading; 
    sendingDataActive = (initialSwitchReading == HIGH); 
    Serial.printf("Kondisi Awal Limit Switch: GPIO2 %s -> Pengiriman %s\n",
                  initialSwitchReading == LOW ? "LOW (NC)" : "HIGH (Open)",
                  sendingDataActive ? "AKTIF" : "NONAKTIF");

    // --- Inisialisasi Sensor Lokal (OLED, MPU, GPS) ---
    Wire.begin(OLED_SDA, OLED_SCL); 
    WireMPU.begin(MPU_SDA, MPU_SCL); 

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { Serial.println(F("SSD1306 gagal")); }
    else { display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE); display.setCursor(0,0); display.println("OLED OK"); display.display(); Serial.println("OLED OK"); delay(200); }

    Serial1.begin(115200, SERIAL_8N1, GPS_RX, GPS_TX); 
    if (!myGNSS.begin(Serial1)) { Serial.println("GPS tidak terdeteksi."); }
    else { 
        bool freqSet = false;
        if (myGNSS.setNavigationFrequency(5)) { Serial.println("GPS OK, Nav Freq 5 Hz"); freqSet = true; } // Coba 5Hz
        else if (myGNSS.setNavigationFrequency(1)) { Serial.println("GPS OK, Nav Freq 1 Hz"); freqSet = true; } // Fallback ke 1Hz
        else { Serial.println("Gagal set Nav Freq GPS."); }
    } 
    delay(200);

    mpu.initialize();
    if (!mpu.testConnection()) { Serial.println("MPU6050 tidak terdeteksi."); }
    else { Serial.println("MPU6050 OK"); }
    delay(200);
    // --- Akhir Inisialisasi Sensor Lokal ---


    // --- Inisialisasi SIM800L ---
    Serial.println("Inisialisasi SIM800L...");
    #ifdef SIM800_RST_PIN
      pinMode(SIM800_RST_PIN, OUTPUT);
      digitalWrite(SIM800_RST_PIN, HIGH); delay(100);
      digitalWrite(SIM800_RST_PIN, LOW); delay(100);
      digitalWrite(SIM800_RST_PIN, HIGH); delay(1000); 
    #endif

    simSerial.begin(9600, SERIAL_8N1, SIM800_RX_PIN, SIM800_TX_PIN); 
    Serial.println("Serial ke SIM800L dimulai pada 9600 baud.");
    delay(1000); 

    Serial.println("Merestart dan menginisialisasi modem (mencoba maks 3 kali)...");
    bool modemInitialized = false;
    for (int retryCount = 0; retryCount < 3; retryCount++) {
        Serial.printf("Percobaan inisialisasi modem ke-%d...\n", retryCount + 1);
        if (modem.restart()) { 
            modemInitialized = true;
            Serial.println("Modem berhasil direstart dan diinisialisasi.");
            break; 
        }
        Serial.println("Gagal, mencoba lagi setelah jeda...");
        delay(2000 + (retryCount * 1000)); 
    }

    if (!modemInitialized) {
         Serial.println("\nMODEM GAGAL INISIALISASI setelah beberapa percobaan!");
         display.clearDisplay(); 
         display.setCursor(0,0); display.print("MODEM ERR!"); 
         display.setCursor(0,8); display.print("CHECK PWR/PIN");
         display.display();
         while(1) { delay(1000); } 
         return; 
    }
    
    String modemInfo = modem.getModemInfo();
    Serial.print("Info Modem: "); Serial.println(modemInfo);
    // --- Akhir Inisialisasi SIM800L ---

    // Inisialisasi P untuk filter
    P_roll[0][0] = 0.5; P_roll[1][1] = 0.5; 
    P_pitch[0][0] = 0.5; P_pitch[1][1] = 0.5;
    kf_speed_P[0][0] = 0.5; kf_speed_P[1][1] = 0.2; 
    
    mqtt.setServer(mqtt_broker, mqtt_port);
    mqtt.setCallback(mqttCallback); 

    display.clearDisplay(); display.setCursor(0, 0);
    display.println("Setup Selesai!"); 
    display.println(sendingDataActive ? "Send: ON (HIGH)" : "Send: OFF (LOW)");
    display.display();
    Serial.println("Setup Selesai. Sistem siap.");
    
    lastUpdateTime = micros(); 
    lastLogTimeMQTT = millis();    
    delay(300); 
}

// =======================================================
//             LOOP UTAMA 
// =======================================================
void loop() {
    unsigned long currentMicros = micros();
    dt = (currentMicros - lastUpdateTime) / 1000000.0f;
    lastUpdateTime = currentMicros; 

    // --- Handle Input Limit Switch ---
    bool previousSendingDataActiveState = sendingDataActive; 
    bool currentRawButtonReading = digitalRead(BUTTON_PIN); 

    if (currentRawButtonReading != lastButtonState) {
        lastButtonPressTime = millis(); 
    }

    if ((millis() - lastButtonPressTime) > DEBOUNCE_DELAY_MS) {
        if (currentRawButtonReading == HIGH) { 
            sendingDataActive = true; 
        } else { 
            sendingDataActive = false; 
        }

        if (sendingDataActive != previousSendingDataActiveState) { 
            Serial.printf("=== Limit Switch State Change! Pengiriman Data: %s (GPIO2: %s) ===\n",
                          sendingDataActive ? "AKTIF" : "NONAKTIF",
                          currentRawButtonReading == HIGH ? "HIGH" : "LOW");
        }
    }
    lastButtonState = currentRawButtonReading; 

    // --- Baca Sensor & Proses Data (GPS, MPU, Filter) ---
    bool gps_updated = myGNSS.checkUblox(); 
    mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);
    
    accX_mss = (ax_raw * ACCEL_RAW_TO_MSS) - ax_offset_mss;
    accY_mss = (ay_raw * ACCEL_RAW_TO_MSS) - ay_offset_mss;
    accZ_mss = (az_raw * ACCEL_RAW_TO_MSS) - az_offset_mss;
    gyroX_dps = (gx_raw * GYRO_RAW_TO_DPS) - gx_offset_dps;
    gyroY_dps = (gy_raw * GYRO_RAW_TO_DPS) - gy_offset_dps;
    gyroZ_dps = (gz_raw * GYRO_RAW_TO_DPS) - gz_offset_dps;
    gyroX_rps = gyroX_dps * DEG_TO_RAD;
    gyroY_rps = gyroY_dps * DEG_TO_RAD;
    gyroZ_rps = gyroZ_dps * DEG_TO_RAD;

    roll_acc_deg = atan2(accY_mss, sqrt(accX_mss * accX_mss + accZ_mss * accZ_mss)) * RAD_TO_DEG;
    pitch_acc_deg = atan2(-accX_mss, sqrt(accY_mss * accY_mss + accZ_mss * accZ_mss)) * RAD_TO_DEG;
    kalmanUpdateGeneric(roll_acc_deg, gyroX_dps, dt, angle_roll, bias_roll, P_roll, 0.001f, 0.003f, 0.03f);
    kalmanUpdateGeneric(pitch_acc_deg, gyroY_dps, dt, angle_pitch, bias_pitch, P_pitch, 0.001f, 0.003f, 0.03f);
    
    validGpsCOG = false;
    if (gps_updated && myGNSS.getPVT()) { 
        fix_type = myGNSS.getFixType(); sats = myGNSS.getSIV(); 
        oled_year=myGNSS.getYear(); oled_month=myGNSS.getMonth(); oled_day=myGNSS.getDay();
        oled_hour=myGNSS.getHour(); oled_minute=myGNSS.getMinute(); oled_second=myGNSS.getSecond();
        if (fix_type >= 2) {
            lat_deg=myGNSS.getLatitude()/10000000.0; lon_deg=myGNSS.getLongitude()/10000000.0; alt_m=myGNSS.getAltitudeMSL()/1000.0;
            spd_mps=myGNSS.getGroundSpeed()/1000.0;
            if (myGNSS.packetUBXNAVPVT != nullptr) {
                velN_mps=myGNSS.packetUBXNAVPVT->data.velN/1000.0;
                velE_mps=myGNSS.packetUBXNAVPVT->data.velE/1000.0;
                velD_mps=myGNSS.packetUBXNAVPVT->data.velD/1000.0;
                if (spd_mps > MIN_SPEED_FOR_COG && (abs(velN_mps)>1e-6 || abs(velE_mps)>1e-6)) {
                    gpsCOG_rad=atan2(velE_mps, velN_mps); validGpsCOG=true;
                }
            } else { velN_mps=0; velE_mps=0; velD_mps=0; }
        } else { spd_mps = 0; }
    }
    if (oled_year > 2000 && fix_type >=2) { 
        int serial_hour=oled_hour+7; int serial_day=oled_day;
        if(serial_hour>=24){serial_hour-=24; serial_day++;}
        snprintf(wibTimeSerial,sizeof(wibTimeSerial),"%04d-%02d-%02d %02d:%02d:%02d",oled_year,oled_month,serial_day,serial_hour,oled_minute,oled_second);
    } else if (fix_type < 2 && oled_year > 1980) { snprintf(wibTimeSerial, sizeof(wibTimeSerial), "Acquiring..."); }
    else { snprintf(wibTimeSerial, sizeof(wibTimeSerial), "No GPS Time"); }

    float forward_acceleration_estimate = accX_mss; 
    kalmanUpdateGeneric(spd_mps, forward_acceleration_estimate, dt, kf_estimated_speed_mps, kf_speed_accel_bias_mps2, kf_speed_P, KF_SPEED_Q_SPEED, KF_SPEED_Q_BIAS, KF_SPEED_R_MEASURE);
    if (kf_estimated_speed_mps < 0) kf_estimated_speed_mps = 0; 

    float predictedYaw = estYawRad + gyroZ_rps * dt; predictedYaw = normalizeAngle(predictedYaw);
    if (validGpsCOG) { float errorYaw = gpsCOG_rad - predictedYaw; errorYaw = normalizeAngle(errorYaw); estYawRad = predictedYaw + (1.0 - YAW_FILTER_ALPHA) * errorYaw; estYawRad = normalizeAngle(estYawRad);
    } else { estYawRad = predictedYaw; }

    float gps_direct_speed_kmh = spd_mps * 3.6;
    float kalman_filtered_speed_kmh = kf_estimated_speed_mps * 3.6;
    // --- Akhir Baca Sensor & Proses Data ---


    // --- Logika Koneksi GPRS dan MQTT ---
    if (!modem.isNetworkConnected()) { // Cek koneksi jaringan dulu
        Serial.println("Jaringan seluler tidak tersambung. Mencoba menyambungkan...");
        display.setCursor(0,0); display.print("NET Conn.."); display.display(); 
        if (modem.waitForNetwork(180000L, true)) { // Tunggu hingga 3 menit, true untuk menampilkan progress
             Serial.println("Jaringan seluler tersambung.");
             display.setCursor(0,0); display.print("NET OK    "); display.display();
        } else {
            Serial.println("Gagal menyambung ke jaringan seluler.");
            display.setCursor(0,0); display.print("NET Fail  "); display.display();
            delay(1000); // Jeda sebelum retry
            // return; // Mungkin tidak perlu keluar dari loop, biarkan coba lagi
        }
    }
    
    if (modem.isNetworkConnected() && !modem.isGprsConnected()) {
        Serial.println("GPRS tidak tersambung. Mencoba menyambungkan GPRS...");
        display.setCursor(0,0); display.print("GPRS Conn."); display.display(); 
        if (modem.gprsConnect(apn, gprsUser, gprsPass)) {
            Serial.println("GPRS tersambung.");
            display.setCursor(0,0); display.print("GPRS OK   "); display.display(); 
        } else {
            Serial.println("Gagal menyambungkan GPRS.");
            display.setCursor(0,0); display.print("GPRS Fail "); display.display(); 
            delay(1000); 
        }
    }

    if (modem.isGprsConnected() && !mqtt.connected()) {
        unsigned long now = millis();
        if (now - lastMqttConnectAttempt > MQTT_RECONNECT_INTERVAL) { 
            lastMqttConnectAttempt = now;
            connectMQTT();
        }
    }
    
    mqtt.loop(); 

    // --- Pengiriman Data MQTT ---
    if (sendingDataActive && mqtt.connected() && (millis() - lastLogTimeMQTT >= LOGGING_INTERVAL_MS)) {
        if (fix_type >= 2) { 
            String jsonPayload = createPeriodicJson(gps_direct_speed_kmh, kalman_filtered_speed_kmh, dt * 1000.0f);
            if (jsonPayload.length() > 0) {
                Serial.print("Mempublikasikan pesan MQTT ke topik "); Serial.println(mqtt_topic_publish);
                if(mqtt.publish(mqtt_topic_publish, jsonPayload.c_str())) {
                    Serial.println("Pesan MQTT berhasil dipublikasikan.");
                } else {
                    Serial.println("Gagal mempublikasikan pesan MQTT.");
                }
            } else {
                Serial.println("Gagal membuat JSON untuk MQTT (string kosong)!");
            }
        } else {
             Serial.println("Log MQTT dilewati: Belum ada GPS fix.");
        }
        lastLogTimeMQTT = millis();
    }


    // --- Output ke Serial Monitor (dikurangi frekuensinya) ---
    static unsigned long lastSerialPrintTime = 0;
    if (millis() - lastSerialPrintTime > 2000) { 
        Serial.println("---------------------------------------------------");
        Serial.printf("dt: %.1fms | Sats:%d | Fix:%s | Send:%s (Switch:%s)\n",
            dt * 1000.0f, sats, getFixTypeString(fix_type).c_str(), 
            sendingDataActive ? "ON" : "OFF", digitalRead(BUTTON_PIN) == LOW ? "LOW(NC)" : "HIGH(Open)");
        Serial.printf("RPY(deg): R:%.1f P:%.1f Y:%.1f\n", angle_roll, angle_pitch, estYawRad * RAD_TO_DEG);
        Serial.printf("GPS Spd: %.1f kmh | KF Spd: %.1f kmh\n", gps_direct_speed_kmh, kalman_filtered_speed_kmh);
        Serial.printf("Waktu Lokal: %s | MQTT: %s | GPRS: %s | NET: %s\n", 
                        wibTimeSerial, 
                        mqtt.connected() ? "Conn" : "Disc",
                        modem.isGprsConnected() ? "Conn" : "Disc",
                        modem.isNetworkConnected() ? "Conn" : "Disc");
        lastSerialPrintTime = millis();
    }
    
    // --- Output ke OLED Display ---
    display.clearDisplay();
    display.setTextSize(1); display.setTextColor(SSD1306_WHITE);

    display.setCursor(0, 0);
    String fixStrOLED = getFixTypeString(fix_type);
    display.printf("Fix:%s S:%d %s", fixStrOLED.c_str(), sats, sendingDataActive ? "SND" : "OFF");


    display.setCursor(0, 8);
    if (oled_year > 2000 && fix_type >=2) { 
        display.printf("%02d/%02d/%04d", oled_day, oled_month, oled_year);
    } else {
        display.print("Tgl: N/A");
    }
    
    display.setCursor(0, 16);
    if (oled_year > 2000 && fix_type >=2) { 
        int display_hour_wib = oled_hour + 7; 
        if (display_hour_wib >= 24) {
            display_hour_wib -= 24;
        }
        display.printf("%02d:%02d:%02d %s", display_hour_wib, oled_minute, oled_second, mqtt.connected() ? "MQ" : "DM");
    } else {
        display.print("Waktu: N/A");
    }

    display.setCursor(0, 24);
    display.printf("R%.0f P%.0f Y%.0f", angle_roll, angle_pitch, estYawRad * RAD_TO_DEG);
    
    display.display();
}