import serial
import time
import json

PORT = "COM7"
BAUD = 1000000 # fake baudrate when use USB CDC

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
    data = {"T":202,"line":1,"text":"Hello, world!","update":1}
    send_json(data)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nexit")
        ser.close()
