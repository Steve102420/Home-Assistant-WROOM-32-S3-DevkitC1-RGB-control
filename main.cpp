#include <Arduino.h>
#include <Credentials.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>
// WiFi + MQTT adatok
const char *ssid = WIFI_SSID;
const char *password = WIFI_PASSWORD;
const char *mqtt_server = MQTT_SERVER_IP;
const char *mqttUser = MQTT_SERVER_USERNAME;
const char *mqttPsw = MQTT_SERVER_PASSWORD;
int mqttPort = MQTT_SERVER_PORT;

WiFiClient WiFiClient;
PubSubClient mqttPubSub(WiFiClient);

// WS2812 LED beállítások
#define LED_PIN 48
#define LED_COUNT 1
#define GAMMA 2.2 // tipikus gamma érték 2.0 - 2.8 között
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
// Ramp paraméterek
#define FADE_STEP_DELAY 10 // ms, két lépés közötti idő
#define FADE_STEP_SIZE 1   // 1..5 tipikusan, mekkora lépést tegyen brightness-ben

// Állapotváltozók
bool ledEnabled = false;
int brightness = 255;
int colorR = 255, colorG = 255, colorB = 255;
// Belső változók a rampeléshez
int currentBrightness = 0;  // tényleges fényerő, amit a LED használ
int targetBrightness = 255; // ide akarunk eljutni
unsigned long lastFadeStep = 0;

//----------------------------------------------------------------------------------------
void SetupWifi(void);
void MqttReconnect(void);
void MqttReceiverCallback(char *topic, byte *inFrame, unsigned int length);
void updateLed(void);
void publishState(void);
uint8_t gammaCorrect(uint8_t value);
//----------------------------------------------------------------------------------------

void setup()
{
    Serial.begin(115200);
    delay(500);

    SetupWifi();

    mqttPubSub.setServer(mqtt_server, mqttPort);
    mqttPubSub.setCallback(MqttReceiverCallback);

    // WS2812 inicializálás
    strip.begin();
    strip.show(); // minden LED off
    strip.setBrightness(255);

    // teszt villogás
    strip.setPixelColor(0, strip.Color(10, 3, 3));
    strip.show();
    delay(500);
    strip.clear();
    strip.show();
    delay(500);

    updateLed();
    publishState();
}

void loop()
{
    if (!mqttPubSub.connected())
    {
        MqttReconnect();
    }
    mqttPubSub.loop();

    if (WiFi.status() != WL_CONNECTED)
    {
        SetupWifi();
    }

    // Rampelés kezelése
    unsigned long now = millis();
    if (now - lastFadeStep >= FADE_STEP_DELAY)
    {
        lastFadeStep = now;

        if (currentBrightness < targetBrightness)
        {
            currentBrightness += FADE_STEP_SIZE;
            if (currentBrightness > targetBrightness)
                currentBrightness = targetBrightness;
            updateLed();
        }
        else if (currentBrightness > targetBrightness)
        {
            currentBrightness -= FADE_STEP_SIZE;
            if (currentBrightness < targetBrightness)
                currentBrightness = targetBrightness;
            updateLed();
        }
    }
}

void SetupWifi(void)
{
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);

    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 20)
    {
        delay(500);
        Serial.print(".");
        retries++;
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("WiFi connected");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
    }
    else
    {
        Serial.println("WiFi NOT connected!");
    }
}
void MqttReconnect(void)
{
    while (!mqttPubSub.connected())
    {
        Serial.print("Attempting MQTT connection...");
        if (mqttPubSub.connect("csepelhome_esp32_rgb", mqttUser, mqttPsw)) // új device ID
        {
            Serial.println("connected");
            mqttPubSub.subscribe("csepelhome/light/esp32rgb/set");
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(mqttPubSub.state());
            Serial.println(" try again in 2 seconds");
            delay(2000);
        }
    }
}
void MqttReceiverCallback(char *topic, byte *inFrame, unsigned int length)
{
    String message;
    for (int i = 0; i < length; i++)
    {
        message += (char)inFrame[i];
    }
    message.trim();
    Serial.printf("Message on [%s]: %s\n", topic, message.c_str());

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, message);
    if (error)
    {
        Serial.println("JSON parse error");
        return;
    }

    if (doc["state"].is<const char *>())
    {
        ledEnabled = (String(doc["state"].as<const char *>()) == "ON");
    }

    if (doc["brightness"].is<int>())
    {
        int val = doc["brightness"].as<int>();
        targetBrightness = constrain(val, 0, 255);
        if (targetBrightness > 0)
            ledEnabled = true;
    }

    if (doc["color"].is<JsonObject>())
    {
        colorR = doc["color"]["r"] | colorR;
        colorG = doc["color"]["g"] | colorG;
        colorB = doc["color"]["b"] | colorB;
    }

    updateLed();
    publishState();
}
void updateLed(void)
{
    uint8_t r, g, b;

    if (ledEnabled && currentBrightness > 0)
    {
        r = gammaCorrect((colorR * currentBrightness) / 255);
        g = gammaCorrect((colorG * currentBrightness) / 255);
        b = gammaCorrect((colorB * currentBrightness) / 255);

        strip.setBrightness(currentBrightness);
        strip.setPixelColor(0, strip.Color(r, g, b));
    }
    else
    {
        strip.clear();
    }
    strip.show();
}
void publishState(void)
{
    JsonDocument doc;
    doc["state"] = ledEnabled ? "ON" : "OFF";
    doc["brightness"] = brightness;

    JsonObject color = doc["color"].to<JsonObject>();
    color["r"] = colorR;
    color["g"] = colorG;
    color["b"] = colorB;

    String json;
    serializeJson(doc, json);
    mqttPubSub.publish("csepelhome/light/esp32rgb/state", json.c_str(), true);
}
uint8_t gammaCorrect(uint8_t value)
{
    float normalized = (float)value / 255.0;   // 0.0 – 1.0 közé normalizál
    float corrected = pow(normalized, GAMMA);  // gammakorrekció
    return (uint8_t)(corrected * 255.0 + 0.5); // visszaalakítás 0–255 közé
}
