# mqtt_connect_test.py
import ssl
import time
import paho.mqtt.client as mqtt

HOST = "192.168.18.3"
PORT = 8883
USERNAME = "apiControl"
PASSWORD = "sch123"
TOPIC = "itk/dt/robot/pose"  # use the exact topic your broker exposes

def on_connect(client, userdata, flags, rc):
    print(f"Connected with rc={rc}")
    client.subscribe(TOPIC)
    print(f"Subscribed to {TOPIC}")

def on_message(client, userdata, msg):
    print(f"{msg.topic}: {msg.payload[:120]}")

client = mqtt.Client()
client.username_pw_set(USERNAME, PASSWORD)

# TLS enabled, skip certificate validation (as in your explorer setup)
ctx = ssl.SSLContext(ssl.PROTOCOL_TLSv1_2)
client.tls_set_context(ctx)
client.tls_insecure_set(True)

client.on_connect = on_connect
client.on_message = on_message

print(f"Connecting to {HOST}:{PORT} ...")
client.connect(HOST, PORT, keepalive=60)

client.loop_start()
time.sleep(10)  # watch for messages for 10 seconds
client.loop_stop()
client.disconnect()
print("Done.")