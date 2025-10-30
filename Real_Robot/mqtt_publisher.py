#!/usr/bin/env python3
import ssl, json, time, uuid, math
import paho.mqtt.client as mqtt

# Robot broker connection info
HOST = "192.168.18.3"
PORT = 8883
USERNAME = "apiControl"
PASSWORD = "sch123"

# MQTT topics (these are fixed by Omron’s MQTT bridge)
POSE_TOPIC     = "itk/dt/robot/pose"      # publishes current pose {x,y in mm, th in mrad, upd in ms}
STATUS_TOPIC   = "itk/dt/robot/status"    # publishes {"status": "Driving"/...}
ARCL_REQ_TOPIC = "itk/cmd/arcl/req"       # publish {"id":"..","command":"..."} → robot executes ARCL
ARCL_RES_ANY   = "itk/cmd/arcl/res/+"     # responses to requests
ARCL_UPDATE    = "itk/dt/arcl/update"     # live ARCL text messages ("Going to goal...", "Arrived")

# Distance to move ahead (3 m)
DIST_MM = 3000

# Store latest pose, plus flags
latest = {"x_mm": None, "y_mm": None, "th_rad": None, "upd_ms": None}
sent = False    # we only want to send one gotoPoint once
req_id = None   # ARCL request ID (correlates req/res)

def send_arcl(client, command: str, req_id_override: str = None):
    """
    Helper to publish an ARCL command to the robot via MQTT.
    Builds the JSON payload with a unique ID and the command string.
    """
    global req_id
    # generate unique request ID (hex string)
    req_id = req_id_override or uuid.uuid4().hex[:12]
    payload = {"id": req_id, "command": command}
    # publish to ARCL request topic
    client.publish(ARCL_REQ_TOPIC, json.dumps(payload), qos=2)
    print(f"[ARCL] -> {command} (id={req_id})")

def on_connect(c, u, f, rc, p=None):
    """
    Called once MQTT connects successfully.
    Subscribes to the pose, status, and ARCL result/update topics.
    """
    print(f"Connected rc={rc}")
    c.subscribe(POSE_TOPIC, qos=1)
    c.subscribe(STATUS_TOPIC, qos=1)
    c.subscribe(ARCL_RES_ANY, qos=1)
    c.subscribe(ARCL_UPDATE, qos=1)

def on_message(c, u, msg):
    """
    Called whenever a subscribed topic publishes something.
    Handles pose, status, ARCL results, and ARCL updates.
    """
    global sent
    if msg.topic == POSE_TOPIC:
        # got a pose update → decode JSON
        try:
            d = json.loads(msg.payload.decode("utf-8"))
            x_mm = float(d.get("x", 0.0))
            y_mm = float(d.get("y", 0.0))
            th_mrad = float(d.get("th", 0.0))   # heading in milliradians
            th_rad = th_mrad * 1e-3             # convert to radians

            # store latest pose
            latest.update({"x_mm": x_mm, "y_mm": y_mm,
                           "th_rad": th_rad, "upd_ms": int(d.get("upd", 0))})

            # If we haven’t already sent the gotoPoint
            if not sent and all(v is not None for v in (latest["x_mm"], latest["y_mm"], latest["th_rad"])):
                # compute 3 m ahead in current heading
                x_t = latest["x_mm"] + DIST_MM * math.cos(latest["th_rad"])
                y_t = latest["y_mm"] + DIST_MM * math.sin(latest["th_rad"])
                th_deg = latest["th_rad"] * 180.0 / math.pi  # convert heading to degrees
                # Build ARCL gotoPoint command string
                cmd = f"gotoPoint {int(round(x_t))} {int(round(y_t))} {int(round(th_deg))}"
                # Send it
                send_arcl(c, cmd)
                sent = True  # mark as done so we don’t keep sending

        except Exception as e:
            print("Bad pose message:", e)

    elif msg.topic.startswith("itk/cmd/arcl/res/"):
        # Got a structured ARCL response to our command
        try:
            d = json.loads(msg.payload.decode("utf-8"))
            print(f"[ARCL RES] {d}")
        except Exception:
            # if not valid JSON, just print raw
            print(f"[ARCL RES] {msg.payload.decode('utf-8', 'ignore')}")

    elif msg.topic == ARCL_UPDATE:
        # Text stream from ARCL (“Going to goal...”, “Arrived...”, etc.)
        txt = msg.payload.decode("utf-8", "ignore").strip()
        if txt:
            print(f"[ARCL UPDATE] {txt}")

    elif msg.topic == STATUS_TOPIC:
        # Robot status (“Driving”, “Idle”, etc.)
        try:
            d = json.loads(msg.payload.decode("utf-8"))
            print(f"[STATUS] {d.get('status')}")
        except Exception:
            pass

def main():
    # Set up MQTT client with TLS
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    c.username_pw_set(USERNAME, PASSWORD)
    ctx = ssl.create_default_context(ssl.Purpose.SERVER_AUTH)
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    c.tls_set_context(ctx)
    c.tls_insecure_set(True)

    # Assign our callbacks
    c.on_connect = on_connect
    c.on_message = on_message

    print(f"Connecting to {HOST}:{PORT} ...")
    c.connect(HOST, PORT, keepalive=60)

    try:
        # enter loop to handle incoming/outgoing MQTT traffic
        c.loop_forever()
    except KeyboardInterrupt:
        print("Exiting.")
    finally:
        c.disconnect()

if __name__ == "__main__":
    main()
