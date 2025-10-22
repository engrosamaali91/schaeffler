#!/usr/bin/env python3
import ssl, json, csv, time
import paho.mqtt.client as mqtt

# --- broker / topics ---
HOST = "192.168.18.3"
PORT = 8883
USERNAME = "apiControl"
PASSWORD = "sch123"

POSE_TOPIC   = "itk/dt/robot/pose"    # {x:mm, y:mm, th:mrad, upd:ms}
STATUS_TOPIC = "itk/dt/robot/status"  # {"type":"RobotStatus", "status": "...", "mode":"...", ...}

# --- config ---
MACRO_NAME   = "setSpeed"              # exact macro name to watch in 'mode'
CSV_FILENAME = f"real_log_{MACRO_NAME}_{int(time.time())}.csv"

# --- csv ---
f = open(CSV_FILENAME, "w", newline="")
w = csv.writer(f)
w.writerow(["t","x","y","yaw"])        # KPI-ready

# --- state ---
logging_active = False     # true only while mode == Executing macro <name>
t0_ms = None               # zero-time anchor (ms) at macro start
last_upd = None            # to dedup pose messages by 'upd'

def on_connect(c, u, fflags, rc, p=None):
    print(f"Connected rc={rc}. Subscribing…  → {CSV_FILENAME}")
    c.subscribe([(POSE_TOPIC,1),(STATUS_TOPIC,1)])

def on_message(c, u, msg):
    global logging_active, t0_ms, last_upd

    if msg.topic == STATUS_TOPIC:
        try:
            d = json.loads(msg.payload.decode("utf-8"))
            mode = str(d.get("mode", ""))
            # Start when we see: Executing macro <MACRO_NAME>
            if mode == f"Executing macro {MACRO_NAME}":
                if not logging_active:  # only on the first "start"
                    print(f"[mode] start: {mode}")
                    logging_active = True
                    t0_ms = None
                    last_upd = None

            # Stop when: Completed macro <MACRO_NAME>
            elif mode == f"Completed macro {MACRO_NAME}":
                print(f"[mode] stop:  {mode}")
                logging_active = False
        except Exception as e:
            print("Bad status message:", e)
        return

    if msg.topic == POSE_TOPIC and logging_active:
        try:
            d = json.loads(msg.payload.decode("utf-8"))
            upd = int(d.get("upd", 0))           # robot time in ms
            if upd == 0 or upd == last_upd:
                return
            last_upd = upd

            if t0_ms is None:
                t0_ms = upd
            t = (upd - t0_ms) / 1000.0           # seconds since macro start

            x = float(d.get("x", 0.0)) / 1000.0  # mm → m
            y = float(d.get("y", 0.0)) / 1000.0
            yaw = float(d.get("th", 0.0)) / 1000.0  # mrad → rad

            w.writerow([f"{t:.3f}", x, y, yaw])
            f.flush()
        except Exception as e:
            print("Bad pose message:", e)

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

    print(f"Connecting to {HOST}:{PORT} …")
    c.connect(HOST, PORT, keepalive=60)
    try:
        c.loop_forever()
    except KeyboardInterrupt:
        pass
    finally:
        c.disconnect()
        f.close()
        print(f"Saved {CSV_FILENAME}")

if __name__ == "__main__":
    main()
