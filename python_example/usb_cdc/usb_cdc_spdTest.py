import serial
import time
import json

PORT = "COM15"
BAUD = 1000000 # fake baudrate when use USB CDC

TOTAL_MESSAGES = 3000

ser = serial.Serial(PORT, BAUD, timeout=0.5)

def send_json(data: dict):
    json_str = json.dumps(data) + "\n"
    ser.write(json_str.encode("utf-8"))
    print(f">>> Sent: {json_str.strip()}")

def read_response():
    try:
        line = ser.readline().decode("utf-8").strip()
        if line:
            print(f"<<< Received: {line}")
            return json.loads(line)
    except json.JSONDecodeError:
        print("⚠️ JSON error")
    except Exception as e:
        print(f"⚠️ error: {e}")
    return None

def main():
    time.sleep(2)
    # data = {"T":11,"id":1,"pos":2047,"spd":0,"acc":0}
    start_time = time.time()
    for i in range(TOTAL_MESSAGES):
        data = {"T":202,"line":1,"text":str(i)+"     cmdcmdcmd","update":1}
        send_json(data)
        print(f"sending {i} cmds")
        time.sleep(1/300000)
    end_time = time.time()
    elapsed = end_time - start_time

    print(f"finished: {TOTAL_MESSAGES} cmds, time elapsed {elapsed:.3f}s")
    print(f"frequency: {TOTAL_MESSAGES / elapsed:.2f} cmd/s")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nexit")
        ser.close()
