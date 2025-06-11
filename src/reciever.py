
import paho.mqtt.client as mqtt
import iotProjekt_pb2
import json
import os
from datetime import datetime

BROKER = "localhost"
TOPIC = "esp32/iot"
DATA_FILE = "/var/www/html/data.json"

def on_connect(client, userdata, flags, rc):
    print("Po ^b ^eczono z brokerem, kod:", rc)
    client.subscribe(TOPIC)

def on_message(client, userdata, msg):
    print(f"Odebrano wiadomo ^{ ^g z {msg.topic}")
    try:
        iot_msg = iotProjekt_pb2.IoTMessage()
        iot_msg.ParseFromString(msg.payload)

        print(f"Urz ^edzenie: {iot_msg.device_id}")
        print(f"Czas: {iot_msg.timestamp}")
        print(f"Typ czujnika: {iot_msg.sensor_type}")

        data = {
            "device_id": iot_msg.device_id,
            "timestamp": iot_msg.timestamp,
            "datetime": datetime.now().isoformat(),
            "sensor_type": iot_msg.sensor_type,
        }

        if iot_msg.HasField("bme680_data"):
            print("BME680:")
            print(f"  Temperatura: {iot_msg.bme680_data.temperature:.2f}   C")
            print(f"  Ci ^{nienie: {iot_msg.bme680_data.pressure:.2f} hPa")
            print(f"  Wilgotno ^{ ^g: {iot_msg.bme680_data.humidity:.2f} %")

            data["sensor"] = "BME680"
            data["temperature"] = iot_msg.bme680_data.temperature
            data["pressure"] = iot_msg.bme680_data.pressure
            data["humidity"] = iot_msg.bme680_data.humidity

        elif iot_msg.HasField("hcsr04_data"):
            print("HC-SR04:")
            print(f"  Odleg ^bo ^{ ^g: {iot_msg.hcsr04_data.distance_cm:.2f} cm")

            data["sensor"] = "HC-SR04"
            data["distance_cm"] = iot_msg.hcsr04_data.distance_cm

        # Wczytaj poprzednie dane
        all_data = {}
        if os.path.exists(DATA_FILE):
            with open(DATA_FILE, "r") as f:
                try:
                    all_data = json.load(f)
                except Exception:
                    all_data = {}

        # Zaktualizuj dane dla danego urz ^edzenia
        all_data[iot_msg.device_id] = data

        # Zapisz z powrotem do pliku
        with open(DATA_FILE, "w") as f:
            json.dump(all_data, f, indent=2)

        print("Zaktualizowano data.json\n")

    except Exception as e:
        print("B ^b ^ed dekodowania wiadomo ^{ci:", e)

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(BROKER, 1883, 60)
client.loop_forever()

