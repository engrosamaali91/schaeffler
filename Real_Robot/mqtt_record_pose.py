import ssl, json, csv
import paho.mqtt.client as mqtt

HOST = "192.168.18.3"
PORT = 8883
USERNAME = "apiControl"
PASSWORD = "sch123"
POSE_TOPIC = "itk/dt/robot/pose"
STATUS_TOPIC = "itk/dt/robot/status"
CSV_FILENAME = "real_log_for_kpi_when_driving.csv"

csvf = open(CSV_FILENAME, "w", newline="")
w = csv.writer(csvf)
w.writerow(["t", "x", "y", "yaw"])   # format for KPI script

t0_ms = None
last_upd = None
robot_is_driving = False

def on_connect(c, u, f, rc, p=None):
    print(f"Connected rc={rc}; subscribing {POSE_TOPIC} and {STATUS_TOPIC}")
    c.subscribe(POSE_TOPIC, qos=0)
    c.subscribe(STATUS_TOPIC, qos=0)

def on_message(c, u, msg):
    global t0_ms, last_upd, robot_is_driving

    # Handle status messages → flip logging on/off
    if msg.topic == STATUS_TOPIC:
        try:
            d = json.loads(msg.payload.decode("utf-8"))
            status = d.get("status", "")
            print(f"Robot status: {status}")
            robot_is_driving = (status == "Driving")
        except Exception as e:
            print("Bad status message:", e)

    # Handle pose messages → log only while Driving
    elif msg.topic == POSE_TOPIC and robot_is_driving:
        try:
            d = json.loads(msg.payload.decode("utf-8"))
            upd = int(d.get("upd", 0))  # robot time in ms
            if upd == 0 or upd == last_upd:
                return
            last_upd = upd
            if t0_ms is None:
                t0_ms = upd
            t = (upd - t0_ms) / 1000.0            # ms → seconds since start
            x = float(d.get("x", 0.0)) / 1000.0   # mm → m
            y = float(d.get("y", 0.0)) / 1000.0   # mm → m
            yaw = float(d.get("th", 0.0)) / 1000.0  # mrad → rad
            w.writerow([f"{t:.2f}", x, y, yaw])
            csvf.flush()
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
    print(f"Connecting to {HOST}:{PORT} → {CSV_FILENAME}")
    c.connect(HOST, PORT, keepalive=60)
    try:
        c.loop_forever()
    except KeyboardInterrupt:
        pass
    finally:
        c.disconnect()
        csvf.close()

if __name__ == "__main__":
    main()
