#include <WiFi.h>
#include <string>
extern "C" {
#include "esp_system.h"
#include "esp_event.h"
#include "esp_sntp.h"
}
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <OneWire.h>

// -------- Configuration --------
static constexpr char WIFI_SSID[] = "HUAWEI-F161";
static constexpr char WIFI_PASS[] = "lidongpo";

static constexpr char MQTT_HOST[] = "zc6449b7.ala.cn-hangzhou.emqxsl.cn";
static constexpr uint16_t MQTT_PORT = 8883;
static constexpr char MQTT_PUB_TOPIC[] = "/topic/ykGtJMDoqtbuyLTt/metrics";
static constexpr char MQTT_CMD_TOPIC[] = "/topic/ykGtJMDoqtbuyLTt/cmd";
static constexpr char MQTT_USER[] = "user0";
static constexpr char MQTT_PASS[] = "user0";

static constexpr char NTP_SERVER[] = "ntp.aliyun.com";
static constexpr uint64_t NTP_RESYNC_MS = 6ULL * 60 * 60 * 1000;
static constexpr TickType_t NTP_RESYNC_TICKS = NTP_RESYNC_MS / portTICK_PERIOD_MS;  // avoid pdMS_TO_TICKS overflow

// ESP32-S2: pick a bidirectional GPIO away from flash/PSRAM pins for OneWire.
static constexpr gpio_num_t ONEWIRE_PIN = GPIO_NUM_4;  // DS18B20 data pin, needs pull-up
static constexpr gpio_num_t LED_PIN = GPIO_NUM_9;       // onboard LED pin as requested

// TLS CA chain (Encryption Everywhere DV TLS CA - G2 + DigiCert Global Root G2)
static const char EMQX_CA[] = R"EOF(
-----BEGIN CERTIFICATE-----
MIIEqjCCA5KgAwIBAgIQDeD/te5iy2EQn2CMnO1e0zANBgkqhkiG9w0BAQsFADBh
MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3
d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH
MjAeFw0xNzExMjcxMjQ2NDBaFw0yNzExMjcxMjQ2NDBaMG4xCzAJBgNVBAYTAlVT
MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j
b20xLTArBgNVBAMTJEVuY3J5cHRpb24gRXZlcnl3aGVyZSBEViBUTFMgQ0EgLSBH
MjCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAO8Uf46i/nr7pkgTDqnE
eSIfCFqvPnUq3aF1tMJ5hh9MnO6Lmt5UdHfBGwC9Si+XjK12cjZgxObsL6Rg1njv
NhAMJ4JunN0JGGRJGSevbJsA3sc68nbPQzuKp5Jc8vpryp2mts38pSCXorPR+sch
QisKA7OSQ1MjcFN0d7tbrceWFNbzgL2csJVQeogOBGSe/KZEIZw6gXLKeFe7mupn
NYJROi2iC11+HuF79iAttMc32Cv6UOxixY/3ZV+LzpLnklFq98XORgwkIJL1HuvP
ha8yvb+W6JislZJL+HLFtidoxmI7Qm3ZyIV66W533DsGFimFJkz3y0GeHWuSVMbI
lfsCAwEAAaOCAU8wggFLMB0GA1UdDgQWBBR435GQX+7erPbFdevVTFVT7yRKtjAf
BgNVHSMEGDAWgBROIlQgGJXm427mD/r6uRLtBhePOTAOBgNVHQ8BAf8EBAMCAQYw
HQYDVR0lBBYwFAYIKwYBBQUHAwEGCCsGAQUFBwMCMBIGA1UdEwEB/wQIMAYBAf8C
AQAwNAYIKwYBBQUHAQEEKDAmMCQGCCsGAQUFBzABhhhodHRwOi8vb2NzcC5kaWdp
Y2VydC5jb20wQgYDVR0fBDswOTA3oDWgM4YxaHR0cDovL2NybDMuZGlnaWNlcnQu
Y29tL0RpZ2lDZXJ0R2xvYmFsUm9vdEcyLmNybDBMBgNVHSAERTBDMDcGCWCGSAGG
/WwBAjAqMCgGCCsGAQUFBwIBFhxodHRwczovL3d3dy5kaWdpY2VydC5jb20vQ1BT
MAgGBmeBDAECATANBgkqhkiG9w0BAQsFAAOCAQEAoBs1eCLKakLtVRPFRjBIJ9LJ
L0s8ZWum8U8/1TMVkQMBn+CPb5xnCD0GSA6L/V0ZFrMNqBirrr5B241OesECvxIi
98bZ90h9+q/X5eMyOD35f8YTaEMpdnQCnawIwiHx06/0BfiTj+b/XQih+mqt3ZXe
xNCJqKexdiB2IWGSKcgahPacWkk/BAQFisKIFYEqHzV974S3FAz/8LIfD58xnsEN
GfzyIDkH3JrwYZ8caPTf6ZX9M1GrISN8HnWTtdNCH2xEajRa/h9ZBXjUyFKQrGk2
n2hcLrfZSbynEC/pSw/ET7H5nWwckjmAJ1l9fcnbqkU/pf6uMQmnfl0JQjJNSg==
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIIDjjCCAnagAwIBAgIQAzrx5qcRqaC7KGSxHQn65TANBgkqhkiG9w0BAQsFADBh
MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3
d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH
MjAeFw0xMzA4MDExMjAwMDBaFw0zODAxMTUxMjAwMDBaMGExCzAJBgNVBAYTAlVT
MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j
b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IEcyMIIBIjANBgkqhkiG
9w0BAQEFAAOCAQ8AMIIBCgKCAQEAuzfNNNx7a8myaJCtSnX/RrohCgiN9RlUyfuI
2/Ou8jqJkTx65qsGGmvPrC3oXgkkRLpimn7Wo6h+4FR1IAWsULecYxpsMNzaHxmx
1x7e/dfgy5SDN67sH0NO3Xss0r0upS/kqbitOtSZpLYl6ZtrAGCSYP9PIUkY92eQ
q2EGnI/yuum06ZIya7XzV+hdG82MHauVBJVJ8zUtluNJbd134/tJS7SsVQepj5Wz
tCO7TG1F8PapspUwtP1MVYwnSlcUfIKdzXOS0xZKBgyMUNGPHgm+F6HmIcr9g+UQ
vIOlCsRnKPZzFBQ9RnbDhxSJITRNrw9FDKZJobq7nMWxM4MphQIDAQABo0IwQDAP
BgNVHRMBAf8EBTADAQH/MA4GA1UdDwEB/wQEAwIBhjAdBgNVHQ4EFgQUTiJUIBiV
5uNu5g/6+rkS7QYXjzkwDQYJKoZIhvcNAQELBQADggEBAGBnKJRvDkhj6zHd6mcY
1Yl9PMWLSn/pvtsrF9+wX3N3KjITOYFnQoQj8kVnNeyIv/iPsGEMNKSuIEyExtv4
NeF22d+mQrvHRAiGfzZ0JFrabA0UWTW98kndth/Jsw1HKj2ZL7tcu7XUIOGZX1NG
Fdtom/DzMNU+MeKNhJ7jitralj41E6Vf8PlwUHBHQRFXGU7Aj64GxJUTFy8bJZ91
8rGOmaFvE7FBcf6IKshPECBV1/MUReXgRPTqh5Uykw7+U0b6LJ3/iyK5S9kJRaTe
pLiaWN0bfVKfjllDiIGknibVb63dDcY3fe0Dkhvld1927jyNxF1WW6LZZm6zNTfl
MrY=
-----END CERTIFICATE-----
)EOF";

// -------- State & RTOS plumbing --------
enum class DeviceState {
  WIFI_CONNECTING,
  MQTT_CONNECTING,
  TIME_SYNC,
  RUNNING,
  ERROR_STATE,
  REBOOTING,
};

static DeviceState g_state = DeviceState::WIFI_CONNECTING;
static SemaphoreHandle_t g_state_lock;
static TimerHandle_t g_ntp_timer = nullptr;
static volatile bool g_mqtt_connected = false;
static OneWire g_onewire(ONEWIRE_PIN);
static WiFiClientSecure g_tls;
static PubSubClient g_pubsub(g_tls);
static std::string g_device_id;

static void set_state(DeviceState s) {
  xSemaphoreTake(g_state_lock, portMAX_DELAY);
  g_state = s;
  xSemaphoreGive(g_state_lock);
}

static DeviceState get_state() {
  xSemaphoreTake(g_state_lock, portMAX_DELAY);
  DeviceState s = g_state;
  xSemaphoreGive(g_state_lock);
  return s;
}

static const std::string &device_id() {
  if (!g_device_id.empty()) return g_device_id;
  uint32_t chipId = 0;
  for (int i = 0; i < 17; i += 8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }
  char buf[12];
  snprintf(buf, sizeof(buf), "%u", (unsigned int)chipId);
  g_device_id.assign(buf);
  return g_device_id;
}

static std::string metric_timestamp() {
  time_t now = 0;
  time(&now);
  struct tm tm_info;
  char buf[32];
  if (now > 1600000000 && localtime_r(&now, &tm_info)) {
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm_info);
    return std::string(buf);
  }
  // Fallback if time not synced yet; preserve format.
  snprintf(buf, sizeof(buf), "1970-01-01 00:00:00");
  return std::string(buf);
}

// -------- DS18B20 OneWire --------
static float ds18b20_read_c() {
  uint8_t addr[8];
  if (!g_onewire.search(addr)) {
    g_onewire.reset_search();
    return NAN;
  }
  if (OneWire::crc8(addr, 7) != addr[7] || addr[0] != 0x28) {
    g_onewire.reset_search();
    return NAN;
  }

  g_onewire.reset();
  g_onewire.select(addr);
  g_onewire.write(0x44, 1);  // start conversion, parasite power on
  vTaskDelay(pdMS_TO_TICKS(750));

  g_onewire.reset();
  g_onewire.select(addr);
  g_onewire.write(0xBE);  // read scratchpad
  uint8_t data[9];
  for (int i = 0; i < 9; ++i) data[i] = g_onewire.read();
  g_onewire.reset_search();

  int16_t raw = (data[1] << 8) | data[0];
  return raw / 16.0f;
}

// -------- Networking --------
static void sync_time() {
  set_state(DeviceState::TIME_SYNC);
  configTzTime("UTC-8", NTP_SERVER);
  time_t now = 0;
  for (int i = 0; i < 15 && now < 1680000000; ++i) {
    vTaskDelay(pdMS_TO_TICKS(500));
    time(&now);
  }
  if (now < 1680000000) {
    Serial.println("[time] NTP sync failed");
    set_state(DeviceState::ERROR_STATE);
  } else {
    Serial.println("[time] NTP synced");
    set_state(g_mqtt_connected ? DeviceState::RUNNING : DeviceState::MQTT_CONNECTING);
  }
}

static void ntp_timer_cb(TimerHandle_t) {
  sync_time();
}

static void mqtt_on_message(char *topic, uint8_t *payload, unsigned int len) {
  if (!topic) return;
  std::string t(topic);
  std::string p(reinterpret_cast<char *>(payload), reinterpret_cast<char *>(payload) + len);
  if (t == MQTT_CMD_TOPIC && p == "reboot") {
    set_state(DeviceState::REBOOTING);
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
  }
}

static bool mqtt_connect_once() {
  g_tls.stop();
  g_tls.setCACert(EMQX_CA);
  g_tls.setTimeout(15000);
  g_pubsub.setServer(MQTT_HOST, MQTT_PORT);
  g_pubsub.setCallback(mqtt_on_message);

  char client_id[32];
  snprintf(client_id, sizeof(client_id), "esp32-%08lx", (unsigned long)esp_random());
  bool ok = g_pubsub.connect(client_id, MQTT_USER, MQTT_PASS);
  if (!ok) return false;

  g_pubsub.subscribe(MQTT_CMD_TOPIC);
  g_pubsub.publish(MQTT_PUB_TOPIC, "boot");
  return true;
}

static void mqtt_task(void *) {
  set_state(DeviceState::MQTT_CONNECTING);
  for (;;) {
    if (!g_pubsub.connected()) {
      g_mqtt_connected = false;
      if (!mqtt_connect_once()) {
        set_state(DeviceState::MQTT_CONNECTING);
        vTaskDelay(pdMS_TO_TICKS(2000));
        continue;
      }
      g_mqtt_connected = true;
      set_state(DeviceState::RUNNING);
      Serial.println("[mqtt] connected");
    }
    g_pubsub.loop();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

static void wifi_task(void *) {
  set_state(DeviceState::WIFI_CONNECTING);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    if (retries % 10 == 0) Serial.println("[wifi] connecting...");
    vTaskDelay(pdMS_TO_TICKS(500));
    if (++retries % 40 == 0) WiFi.reconnect();
  }
  Serial.println("[wifi] connected");
  sync_time();
  // 单核/S2 也兼容：不用指定核
  xTaskCreate(mqtt_task, "mqtt", 4096, nullptr, 5, nullptr);
  vTaskDelete(nullptr);
}

static void telemetry_task(void *) {
  for (;;) {
    float c = ds18b20_read_c();
    if (isnan(c)) {
      Serial.println("[temp] sensor read failed");
    } else {
      Serial.printf("[temp] %.2f C\n", c);
      if (g_pubsub.connected() && g_mqtt_connected) {
        char value[24];
        snprintf(value, sizeof(value), "%.2f", c);
        std::string msg = std::string("metrics\n") +
                          device_id() + "\n" +
                          "ds18b20\n" +
                          metric_timestamp() + "\n" +
                          "T\n" +
                          value;
        g_pubsub.publish(MQTT_PUB_TOPIC, msg.c_str());
      }
    }
    vTaskDelay(pdMS_TO_TICKS(60 * 1000));
  }
}

static void led_blink_times(int times, int on_ms, int off_ms) {
  for (int i = 0; i < times; ++i) {
    digitalWrite(LED_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(on_ms));
    digitalWrite(LED_PIN, LOW);
    if (i != times - 1) vTaskDelay(pdMS_TO_TICKS(off_ms));
  }
}

static void led_task(void *) {
  if (!digitalPinIsValid(LED_PIN) || !digitalPinCanOutput(LED_PIN)) {
    Serial.println("[led] invalid LED pin");
    vTaskDelete(nullptr);
  }
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  for (;;) {
    switch (get_state()) {
      case DeviceState::WIFI_CONNECTING:
        led_blink_times(1, 200, 200);
        vTaskDelay(pdMS_TO_TICKS(600));
        break;
      case DeviceState::MQTT_CONNECTING:
        led_blink_times(2, 200, 150);
        vTaskDelay(pdMS_TO_TICKS(500));
        break;
      case DeviceState::TIME_SYNC:
        led_blink_times(3, 150, 120);
        vTaskDelay(pdMS_TO_TICKS(400));
        break;
      case DeviceState::RUNNING:
        digitalWrite(LED_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(900));
        digitalWrite(LED_PIN, LOW);
        vTaskDelay(pdMS_TO_TICKS(100));
        break;
      case DeviceState::ERROR_STATE:
        led_blink_times(4, 120, 120);
        vTaskDelay(pdMS_TO_TICKS(300));
        break;
      case DeviceState::REBOOTING:
      default:
        digitalWrite(LED_PIN, LOW);
        vTaskDelay(pdMS_TO_TICKS(300));
        break;
    }
  }
}

// -------- Arduino entry points --------
void setup() {
  delay(2000);  // allow USB CDC to enumerate
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("[boot] start");
  g_state_lock = xSemaphoreCreateMutex();
  set_state(DeviceState::WIFI_CONNECTING);
  pinMode(ONEWIRE_PIN, INPUT_PULLUP);  // ensure DS18B20 bus has a pull-up if external resistor is weak/absent
  xTaskCreate(led_task, "led", 2048, nullptr, 1, nullptr);
  xTaskCreate(wifi_task, "wifi", 4096, nullptr, 5, nullptr);
  xTaskCreate(telemetry_task, "telemetry", 3072, nullptr, 4, nullptr);
  g_ntp_timer = xTimerCreate("ntp", NTP_RESYNC_TICKS, pdTRUE, nullptr, ntp_timer_cb);
  xTimerStart(g_ntp_timer, 0);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
