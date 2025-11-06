#!/usr/bin/env python3
import ssl, json, csv, time, math
import paho.mqtt.client as mqtt

# --- broker / topics ---
HOST = "192.168.18.3"
PORT = 8883
USERNAME = "apiControl"
PASSWORD = "sch123"

POSE_TOPIC   = "itk/dt/robot/pose"    # {x:mm, y:mm, th:deg, upd:ms} <-- Updated comment
STATUS_TOPIC = "itk/dt/robot/status"  # {"type":"RobotStatus", "status":"...", "mode":"..."}

# --- config ---
MACRO_NAME   = "setSpeed"              # macro name to watch in 'mode'
CSV_FILENAME = f"real_log_{MACRO_NAME}_{int(time.time())}.csv"

# --- csv ---
f = open(CSV_FILENAME, "w", newline="")
w = csv.writer(f)
w.writerow(["t","x","y","yaw","vx"])   # match simulation data columns

# --- state ---
logging_active = False
t0_ms = None
last_upd = None

# previous sample for finite differences
prev_t = None
prev_x = None
prev_y = None
prev_yaw = None


def on_connect(c, u, fflags, rc, p=None):
    print(f"Connected rc={rc}. Subscribing…  → {CSV_FILENAME}")
    c.subscribe([(POSE_TOPIC,1), (STATUS_TOPIC,1)])


def on_message(c, u, msg):
    global logging_active, t0_ms, last_upd
    global prev_t, prev_x, prev_y, prev_yaw

    # --- handle status topic (start/stop logging) ---
    if msg.topic == STATUS_TOPIC:
        try:
            d = json.loads(msg.payload.decode("utf-8"))
            mode = str(d.get("mode", ""))
            if mode == f"Executing macro {MACRO_NAME}":
                if not logging_active:
                    print(f"[mode] start: {mode}")
                    logging_active = True
                    t0_ms = None
                    last_upd = None
                    prev_t = prev_x = prev_y = prev_yaw = None
            elif mode == f"Completed macro {MACRO_NAME}":
                if logging_active:
                    print(f"[mode] stop:  {mode}")
                    logging_active = False
        except Exception as e:
            print("Bad status message:", e)
        return

    # --- handle pose topic (record data) ---
    if msg.topic == POSE_TOPIC and logging_active:
        try:
            d = json.loads(msg.payload.decode("utf-8"))
            upd = int(d.get("upd", 0))
            if upd == 0 or upd == last_upd:
                return
            last_upd = upd

            if t0_ms is None:
                t0_ms = upd
            t = (upd - t0_ms) / 1000.0  # seconds since macro start

            x = float(d.get("x", 0.0)) / 1000.0   # mm → m
            y = float(d.get("y", 0.0)) / 1000.0
            
            # --- CORRECTED YAW CONVERSION ---
            # Original was: yaw = float(d.get("th", 0.0)) / 1000.0 # mrad → rad
            # New is:
            yaw = math.radians(float(d.get("th", 0.0))) # deg → rad
            # --------------------------------

            v_forward = ""
            if prev_t is not None:
                dt = t - prev_t
                if dt > 0:
                    dx = x - prev_x
                    dy = y - prev_y
                    dth = yaw - prev_yaw
                    # unwrap yaw for small discontinuities
                    dth = (dth + math.pi) % (2 * math.pi) - math.pi
                    # project displacement into robot's forward axis (body frame)
                    v_forward = (dx * math.cos(prev_yaw) + dy * math.sin(prev_yaw)) / dt

            # write row: t, x, y, yaw, vx
            w.writerow([
                f"{t:.3f}",
                f"{x:.6f}",
                f"{y:.6f}",
                f"{yaw:.6f}",
                (f"{v_forward:.6f}" if v_forward != "" else "")
            ])
            f.flush()

            prev_t, prev_x, prev_y, prev_yaw = t, x, y, yaw

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