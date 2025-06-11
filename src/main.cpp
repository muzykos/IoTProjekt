
#include <WiFi.h>
#include <PubSubClient.h>
#include "iotProjekt.pb.h"
#include <pb_encode.h>
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <SPI.h>
#define BME_CS 5  // lub inny wolny pin


#define SEALEVELPRESSURE_HPA (1013.25)

// ==================== Wi-Fi and MQTT Configuration ====================
const char *ssid = "Pi_AP";
const char *password = "tocozawsze";
const char *mqtt_server = "192.168.4.1"; // IP Raspberry Pi z Mosquitto

WiFiClient espClient;
PubSubClient client(espClient);
const char *mqtt_topic = "esp32/iot";

// ==================== Device and Sensor Configuration ====================
const char *device_id = "esp32-020";

// Zmienna globalna: SensorType_BME680 lub SensorType_HCSR04
int sensor_type = iot_SensorType_BME680;

// BME680
Adafruit_BME680 bme(BME_CS);
// HC-SR04
const int trigPin = 5;
const int echoPin = 18;

// ==================== Callback to encode strings (nanopb) ====================
bool pb_encode_string(pb_ostream_t *stream, const pb_field_t *field, void *const *arg) {
    const char *str = (const char *)(*arg);
    return pb_encode_tag_for_field(stream, field) &&
           pb_encode_string(stream, (const uint8_t *)str, strlen(str));
}

// ==================== MQTT Connection ====================
void reconnect() {
    while (!client.connected()) {
        Serial.print("Łączenie z MQTT...");
        if (client.connect("esp32client")) {
            Serial.println("połączono");
        } else {
            Serial.print("Błąd, rc=");
            Serial.print(client.state());
            Serial.println(" ponowna próba za 5 sek.");
            delay(5000);
        }
    }
}

// ==================== Send IoTMessage via MQTT ====================
void sendMessage(iot_IoTMessage &msg, const char *label) {
    uint8_t buffer[128];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

    if (!pb_encode(&stream, iot_IoTMessage_fields, &msg)) {
        Serial.print("Błąd kodowania ");
        Serial.println(label);
        return;
    }

    client.publish(mqtt_topic, buffer, stream.bytes_written);
    Serial.print("Wysłano przez MQTT: ");
    Serial.println(label);
}

// ==================== Setup ====================
void setup() {
    Serial.begin(460800);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nPołączono z Wi-Fi");

    client.setServer(mqtt_server, 1883);

    if (sensor_type == iot_SensorType_BME680) {
        if (!bme.begin()) {
            Serial.println("Nie wykryto BME680 przez SPI!");
            while (1);
        }        
        bme.setTemperatureOversampling(BME680_OS_8X);
        bme.setHumidityOversampling(BME680_OS_2X);
        bme.setPressureOversampling(BME680_OS_4X);
        bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
        bme.setGasHeater(320, 150);
    } else if (sensor_type == iot_SensorType_HCSR04) {
        pinMode(trigPin, OUTPUT);
        pinMode(echoPin, INPUT);
    }
    

    if (bme.begin()) {
        sensor_type = iot_SensorType_BME680;
    } else {
        sensor_type = iot_SensorType_HCSR04;
    }
}

// ==================== Main Loop ====================
void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    iot_IoTMessage msg = iot_IoTMessage_init_zero;
    msg.device_id.funcs.encode = &pb_encode_string;
    msg.device_id.arg = (void *)device_id;
    msg.timestamp = millis();
    msg.sensor_type = (iot_SensorType)sensor_type;

    if (sensor_type == iot_SensorType_BME680) {
        if (!bme.performReading()) {
            Serial.println("Błąd odczytu z BME680");
            delay(1000);
            return;
        }

        msg.which_payload = iot_IoTMessage_bme680_data_tag;
        msg.payload.bme680_data.temperature = bme.temperature;
        msg.payload.bme680_data.pressure = bme.pressure / 100.0;
        msg.payload.bme680_data.humidity = bme.humidity;

        sendMessage(msg, "BME680");
    } else if (sensor_type == iot_SensorType_HCSR04) {
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);

        long duration = pulseIn(echoPin, HIGH);
        float distance = duration * 0.034 / 2;

        msg.which_payload = iot_IoTMessage_hcsr04_data_tag;
        msg.payload.hcsr04_data.distance_cm = distance;

        Serial.println(distance);
        sendMessage(msg, "HC-SR04");
    }

    delay(5000);
}