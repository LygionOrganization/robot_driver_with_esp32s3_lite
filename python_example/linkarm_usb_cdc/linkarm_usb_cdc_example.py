import serial
import time
import json
import threading

PORT = "COM26"
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

def response_thread(stop_event: threading.Event):
    while not stop_event.is_set():
        read_response()

def main():
    stop_event = threading.Event()
    t = threading.Thread(target=response_thread, args=(stop_event,), daemon=True)
    t.start()

    # time.sleep(1)
    # send_json({"T":108,"id":254,"state":0})

    # while True:
    #     time.sleep(10)

    time.sleep(1)
    data = {"T":131,"pos":[496,514,292,514]}
    send_json(data)
    time.sleep(1)

    # while True:
    #     send_json({"T":138,"xyzg":[200,-250,-90,0],"spd":1.2})
    #     time.sleep(1.5)

    #     send_json({"T":138,"xyzg":[200,-250,-90,-53],"spd":1.2})
    #     time.sleep(1.5)

    #     send_json({"T":138,"xyzg":[200,-250,90,-53],"spd":1.2})
    #     time.sleep(1.5)

    #     send_json({"T":138,"xyzg":[100,-250,90,-53],"spd":1.2})
    #     time.sleep(1.5)

    #     send_json({"T":138,"xyzg":[100,-250,-90,-53],"spd":1.2})
    #     time.sleep(1.5)

    #     send_json({"T":138,"xyzg":[100,-250,-90,0],"spd":1.2})
    #     time.sleep(1.5)

    #     send_json({"T":138,"xyzg":[100,-250,-90,-53],"spd":1.2})
    #     time.sleep(1.5)

    #     send_json({"T":138,"xyzg":[100,-250,90,-53],"spd":1.2})
    #     time.sleep(1.5)

    #     send_json({"T":138,"xyzg":[200,-250,90,-53],"spd":1.2})
    #     time.sleep(1.5)

    #     send_json({"T":138,"xyzg":[200,-250,-90,-53],"spd":1.2})
    #     time.sleep(1.5)

        # send_json({"T":138,"xyzg":[200,-250,-90,0],"spd":1.2})
        # time.sleep(1.5)

    while True:
        send_json({"T":138,"xyzg":[230,100,200,-50],"spd":1.2})
        time.sleep(1.5)

        send_json({"T":138,"xyzg":[230,100,0,0],"spd":1.2})
        time.sleep(1.5)

        send_json({"T":138,"xyzg":[230,-100,0,0],"spd":1.2})
        time.sleep(1.5)

        send_json({"T":138,"xyzg":[230,-100,200,-50],"spd":1.2})
        time.sleep(1.5)

    # while True:
    #     send_json({"T":138,"xyzg":[230,100,200,-50],"spd":1.2})
    #     time.sleep(1.5)

    #     send_json({"T":138,"xyzg":[230,100,200,0],"spd":1.2})
    #     time.sleep(1.5)



if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nexit")
        ser.close()
