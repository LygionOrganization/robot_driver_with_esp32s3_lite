import json
import time
import websocket

ESP32_IP = "192.168.0.103"
PORT = 80
PATH = "/ws"
WS_URL = f"ws://{ESP32_IP}:{PORT}{PATH}"

TOTAL_MESSAGES = 3000

def on_message(ws, message):
    """callback: message recv"""
    pass


def on_error(ws, error):
    """callback: error"""
    print(f"WebSocket error: {error}")


def on_close(ws, close_status_code, close_msg):
    """callback: connection closed"""
    print("connection closed")


def on_open(ws):
    """callback: new connection"""
    print("connected to ESP32 WebSocket")

    start_time = time.time()
    for i in range(TOTAL_MESSAGES):
        data = {"T":202,
            "line":1,
            "text":f"sending {i} cmds",
            "update":1}
        ws.send(json.dumps(data))
        print(f"sending {i} cmds")
        time.sleep(1/6000)
    end_time = time.time()
    elapsed = end_time - start_time

    print(f"finished: {TOTAL_MESSAGES} cmds, time elapsed {elapsed:.3f}s")
    print(f"frequency: {TOTAL_MESSAGES / elapsed:.2f} cmd/s")
    ws.close()

def close_ws():
    ws.close()


if __name__ == "__main__":
    websocket.enableTrace(False)
    ws = websocket.WebSocketApp(
        WS_URL,
        on_message=on_message,
        on_error=on_error,
        on_close=on_close,
        on_open=on_open
    )

    ws.run_forever()