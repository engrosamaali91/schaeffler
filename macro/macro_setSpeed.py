#!/usr/bin/env python3
import ssl, json, uuid
import paho.mqtt.client as mqtt

HOST, PORT = "192.168.18.3", 8883
USERNAME, PASSWORD = "apiControl", "sch123"

ARCL_REQ     = "itk/cmd/arcl/req"
ARCL_RES_ANY = "itk/cmd/arcl/res/+"
ARCL_UPDATE  = "itk/dt/arcl/update"

def send_arcl(client, command: str):
    rid = uuid.uuid4().hex[:12]
    payload = {"id": rid, "command": command}
    client.publish(ARCL_REQ, json.dumps(payload), qos=2)
    print(f"[ARCL ->] {command} (id={rid})")
    return rid

def on_connect(c, u, f, rc, p=None):
    print("Connected.")
    c.subscribe(ARCL_RES_ANY, qos=1)
    c.subscribe(ARCL_UPDATE,  qos=1)

def on_message(c, u, msg):
    if msg.topic.startswith("itk/cmd/arcl/res/"):
        try:
            d = json.loads(msg.payload.decode("utf-8"))
            print(f"[RES] {d}")
        except Exception:
            print(f"[RES] {msg.payload.decode('utf-8','ignore')}")
    elif msg.topic == ARCL_UPDATE:
        txt = msg.payload.decode("utf-8","ignore").strip()
        if txt:
            print(f"[UPD] {txt}")

def main():
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    c.username_pw_set(USERNAME, PASSWORD)

    ctx = ssl.create_default_context(ssl.Purpose.SERVER_AUTH)
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    c.tls_set_context(ctx)
    c.tls_insecure_set(True)

    c.on_connect = on_connect
    c.on_message = on_message

    print(f"Connecting to {HOST}:{PORT} ...")
    c.connect(HOST, PORT, keepalive=60)
    c.loop_start()

    # --- Execute your macro named 'setSpeed' ---
    send_arcl(c, "executeMacro setSpeed")

    # Let updates come in while the robot moves
    try:
        while True:
            pass
    except KeyboardInterrupt:
        print("Exiting.")
        c.loop_stop()
        c.disconnect()

if __name__ == "__main__":
    main()
