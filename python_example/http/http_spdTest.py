import requests
import json
import time

ESP32_IP = "192.168.0.103"
PORT = 80
URL = f"http://{ESP32_IP}:{PORT}/api/cmd"

TOTAL_MESSAGES = 10000

def send_json_command(payload: dict):
    """send JSON data to ESP32"""
    headers = {"Content-Type": "application/json"}
    try:
        response = requests.post(URL, headers=headers, data=json.dumps(payload), timeout=2)
        print(f"status: {response.status_code}")
        print(f"feedback: {response.text}")
    except requests.exceptions.RequestException as e:
        print(f"exception: {e}")

if __name__ == "__main__":
    # example

    start_time = time.time()
    for i in range(TOTAL_MESSAGES):
        data = {"T":202,
                "line":1,
                "text":f"sending {i} cmds",
                "update":1}
        send_json_command(data)
        # time.sleep(1/60)
    end_time = time.time()
    elapsed = end_time - start_time

    print(f"finished: {TOTAL_MESSAGES} cmds, time elapsed {elapsed:.3f}s")
    print(f"frequency: {TOTAL_MESSAGES / elapsed:.2f} cmd/s")